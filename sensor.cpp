// ============================================================================
//  sensor.cpp — BNO085 implementation of the SensorData abstraction
//
//  Library: Adafruit_BNO08x  (depends on Adafruit_BusIO + Adafruit Unified Sensor)
//  Reports enabled:
//    - SH2_ARVR_STABILIZED_RV   (9-axis fused quaternion, magnetometer ON)
//    - SH2_LINEAR_ACCELERATION  (m/s², gravity already removed by the BNO085)
//    - SH2_GYROSCOPE_CALIBRATED (rad/s, on-chip ZRO bias correction applied)
//
//  IMPORTANT: do NOT switch to SH2_GAME_ROTATION_VECTOR — yaw will drift within
//  ~30 s and break golf hip-rotation and ankle-pronation tracking.
//  Use SH2_GYROSCOPE_CALIBRATED, not _UNCALIBRATED — we want bias-corrected dps.
// ============================================================================
#include "sensor.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_BNO08x.h>

// I2C-mode constructor (no INT pin connected on the breakout).
static Adafruit_BNO08x  bno08x(-1);
static sh2_SensorValue_t sensorValue;

// Last-known good values, refreshed when a new report arrives.
static float last_qw = 1.0f, last_qx = 0.0f, last_qy = 0.0f, last_qz = 0.0f;
static float last_ax = 0.0f, last_ay = 0.0f, last_az = 0.0f;
static float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;
static bool  has_quat  = false;
static bool  has_accel = false;
static bool  has_gyro  = false;

// Configure the two reports we need at SAMPLE_RATE_HZ.
static bool enableReports() {
  // Period in microseconds: 100 Hz -> 10000 us
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, REPORT_INTERVAL_US)) {
    Serial.println(F("[sensor] failed to enable ARVR_STABILIZED_RV"));
    return false;
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, REPORT_INTERVAL_US)) {
    Serial.println(F("[sensor] failed to enable LINEAR_ACCELERATION"));
    return false;
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US)) {
    Serial.println(F("[sensor] failed to enable GYROSCOPE_CALIBRATED"));
    return false;
  }
  return true;
}

bool sensorInit() {
  // Wire.begin() and Wire.setClock(...) are owned by the main sketch.
  if (!bno08x.begin_I2C(BNO085_I2C_ADDR, &Wire)) {
    Serial.println(F("[sensor] BNO085 not found at 0x4A"));
    return false;
  }
  if (!enableReports()) return false;
  Serial.println(F("[sensor] BNO085 ready (9-axis ARVR + linear accel + calibrated gyro)"));
  return true;
}

void sensorPrintInfo() {
  // Adafruit_BNO08x exposes product IDs only after begin_I2C; nothing to print
  // here without diving into sh2.h. Kept as a hook for future diagnostics.
  Serial.println(F("[sensor] BNO085 @ I2C 0x4A, ARVR_STABILIZED_RV + LINEAR_ACCEL + GYRO_CAL @ 100 Hz"));
}

SensorData getSensorAngles() {
  SensorData out;
  out.valid = false;

  // The BNO085 occasionally resets (e.g. after I2C glitches). Re-arm reports.
  if (bno08x.wasReset()) {
    Serial.println(F("[sensor] BNO085 was reset, re-enabling reports"));
    enableReports();
  }

  // Drain every pending report this tick — there may be quat AND accel.
  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV: {
        last_qw = sensorValue.un.arvrStabilizedRV.real;
        last_qx = sensorValue.un.arvrStabilizedRV.i;
        last_qy = sensorValue.un.arvrStabilizedRV.j;
        last_qz = sensorValue.un.arvrStabilizedRV.k;
        has_quat = true;
        break;
      }
      case SH2_LINEAR_ACCELERATION: {
        last_ax = sensorValue.un.linearAcceleration.x;
        last_ay = sensorValue.un.linearAcceleration.y;
        last_az = sensorValue.un.linearAcceleration.z;
        has_accel = true;
        break;
      }
      case SH2_GYROSCOPE_CALIBRATED: {
        // SH2 reports gyro in rad/s; SensorData stores °/s so the BLE layer
        // doesn't need to know the sensor's native units.
        static const float RAD_TO_DEG = 57.2957795f;
        last_gx = sensorValue.un.gyroscope.x * RAD_TO_DEG;
        last_gy = sensorValue.un.gyroscope.y * RAD_TO_DEG;
        last_gz = sensorValue.un.gyroscope.z * RAD_TO_DEG;
        has_gyro = true;
        break;
      }
      default:
        break;
    }
  }

  // Only emit a sample once we've seen at least one of each report,
  // so the first BLE packet isn't full of zeros.
  if (has_quat && has_accel && has_gyro) {
    out.timestamp_ms = millis();
    out.qw = last_qw; out.qx = last_qx; out.qy = last_qy; out.qz = last_qz;
    out.ax = last_ax; out.ay = last_ay; out.az = last_az;
    out.gx = last_gx; out.gy = last_gy; out.gz = last_gz;
    out.valid = true;
  }
  return out;
}
