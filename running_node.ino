// ============================================================================
//  running_node.ino — AI Sport Monitor
//  Sprint 1 firmware for the running node (XIAO nRF52840 Sense + BNO085).
//
//  Single sketch, identical binary on both bands. Each node derives a unique
//  BLE name (`SportBand-XXXX`) from its nRF52840 factory DEVICEID at boot;
//  the app's pairing flow assigns left/right roles. Runs the IMU + BLE
//  pipeline at 100 Hz and reports battery percent every 10 s.
//
//  Required Arduino libraries:
//    - Adafruit BNO08x      (and its deps: Adafruit BusIO, Adafruit Unified Sensor)
//    - ArduinoBLE           >= 1.3.0
//
//  Board: "Seeed XIAO BLE Sense - nRF52840"
//  Boards Manager URL:
//    https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
// ============================================================================

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "sensor.h"
#include "ble_service.h"

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------
static uint32_t session_start_ms  = 0;     // resets when a new central connects
static uint32_t last_sample_ms    = 0;
static uint32_t last_battery_ms   = 0;
static bool     was_connected     = false;

// ---------------------------------------------------------------------------
//  Battery — read VBAT through the internal divider on the XIAO nRF52840
// ---------------------------------------------------------------------------
static uint8_t readBatteryPercent() {
  // Enable the on-board voltage divider (P0.14 LOW = ON).
  pinMode(BATTERY_ENABLE_PIN, OUTPUT);
  digitalWrite(BATTERY_ENABLE_PIN, LOW);

  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(ADC_RESOLUTION_BITS);

  // Discard one reading after switching reference (recommended by Nordic).
  // Use a short non-blocking-ish settle (500 µs) instead of delay(2 ms) so we
  // don't drop a 100 Hz sample every 10 s when the battery routine fires.
  (void)analogRead(PIN_VBAT);
  delayMicroseconds(500);
  uint32_t raw = analogRead(PIN_VBAT);

  // raw -> volts at the ADC pin -> battery volts (un-divide).
  float v_adc  = (raw * ADC_REF_VOLTAGE) / (float)ADC_MAX;
  float v_bat  = v_adc * VBAT_DIVIDER_RATIO;

  // Linear map between empty and full.
  float pct_f  = 100.0f * (v_bat - VBAT_EMPTY_V) / (VBAT_FULL_V - VBAT_EMPTY_V);
  if (pct_f < 0.0f)   pct_f = 0.0f;
  if (pct_f > 100.0f) pct_f = 100.0f;

  return (uint8_t)(pct_f + 0.5f);
}

// ---------------------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  // Don't block forever waiting for USB Serial — the node has to run on battery.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 1500) { delay(10); }

  Serial.println();
  Serial.println(F("[main] AI Sport Monitor — running node booting"));

  // I2C must be brought up before sensorInit().
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);   // 400 kHz fast mode — required for 100 Hz

  if (!sensorInit()) {
    Serial.println(F("[main] sensor init failed — halting"));
    while (true) { delay(1000); }
  }

  if (!bleInit()) {
    Serial.println(F("[main] BLE init failed — halting"));
    while (true) { delay(1000); }
  }

  // Prime battery characteristic with a real reading so the central sees a
  // sensible value the moment it connects.
  bleUpdateBatteryPercent(readBatteryPercent());
  last_battery_ms = millis();

  Serial.println(F("[main] ready"));
}

// ---------------------------------------------------------------------------
//  Loop — fixed 100 Hz cadence
// ---------------------------------------------------------------------------
void loop() {
  // Service the BLE stack on every tick (handles connect/disconnect events).
  blePoll();

  // Reset the session-relative timestamp clock on a fresh connection so the
  // first packet after reconnect starts at ts=0.
  bool now_connected = bleConnected();
  if (now_connected && !was_connected) {
    session_start_ms = millis();
    last_sample_ms   = 0;
  }
  was_connected = now_connected;

  uint32_t now = millis();

  // --- 100 Hz sensor stream ------------------------------------------------
  if ((now - last_sample_ms) >= SAMPLE_PERIOD_MS) {
    last_sample_ms = now;

    SensorData s = getSensorAngles();
    if (s.valid && now_connected) {
      bleSendSensor(s, session_start_ms);
    }
  }

  // --- 0.1 Hz battery broadcast -------------------------------------------
  if ((now - last_battery_ms) >= BATTERY_INTERVAL_MS) {
    last_battery_ms = now;
    uint8_t pct = readBatteryPercent();
    bleUpdateBatteryPercent(pct);
    Serial.print(F("[main] battery "));
    Serial.print(pct);
    Serial.println(F("%"));
  }
}
