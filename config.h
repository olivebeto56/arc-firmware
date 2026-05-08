// ============================================================================
//  config.h — AI Sport Monitor / running node
//  XIAO nRF52840 Sense + BNO085 + LiPo 400mAh
//
//  Shared configuration: BLE UUIDs, pin map, sample rate. Identity is built
//  at runtime — both bands run an identical binary.
// ============================================================================
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
//  Node identity — runtime, derived from nRF52840 FICR DEVICEID
// ---------------------------------------------------------------------------
//  Both bands run an identical binary. The BLE local name is built at boot
//  as "SportBand-XXXX" where XXXX are the lowest 16 bits of the chip's
//  factory-burned unique ID (nRF52840 FICR DEVICEID, 64 bits, immutable).
//  The L/R assignment lives in the app's pairing flow ("shake the left band").
#define BLE_NAME_PREFIX  "SportBand-"

// ---------------------------------------------------------------------------
//  Sampling
// ---------------------------------------------------------------------------
//  Sprint 1 target: running at fixed 100 Hz.
//  100 Hz → 10 ms period; BNO085 fusion is configured at the same rate.
static const uint16_t SAMPLE_RATE_HZ      = 100;
static const uint16_t SAMPLE_PERIOD_MS    = 1000 / SAMPLE_RATE_HZ;   // 10 ms
static const uint32_t REPORT_INTERVAL_US  = 1000000UL / SAMPLE_RATE_HZ; // 10000 us

// Battery broadcast cadence (the sensor stream is independent and runs at SAMPLE_RATE_HZ).
static const uint32_t BATTERY_INTERVAL_MS = 10000;   // every 10 s

// ---------------------------------------------------------------------------
//  I2C / BNO085
// ---------------------------------------------------------------------------
//  XIAO nRF52840 Sense default I2C: SDA=D4 (P0.06), SCL=D5 (P0.07)
//  BNO085 (Adafruit #4754) default address with SA0=GND is 0x4A.
static const uint8_t  BNO085_I2C_ADDR     = 0x4A;
static const uint32_t I2C_CLOCK_HZ        = 400000;   // fast mode required for 100 Hz

// ---------------------------------------------------------------------------
//  Battery ADC
// ---------------------------------------------------------------------------
//  XIAO nRF52840 wiring: PIN_VBAT measures VBAT through internal divider,
//  P0_14 must be driven LOW to enable the divider, AR_INTERNAL_3_0 reference.
//
//  IMPORTANT: use the BSP-provided P0_14 macro, NOT a raw pin number 14.
//  In the Seeed XIAO nRF52840 Arduino core, "14" as an Arduino pin number
//  resolves to a different physical pin — only P0_14 maps to the actual nRF
//  GPIO that controls the battery voltage divider.
#define BATTERY_ENABLE_PIN  P0_14             // P0.14, LOW = divider ON
static const float    ADC_REF_VOLTAGE     = 3.0f;     // AR_INTERNAL_3_0
static const float    VBAT_DIVIDER_RATIO  = (1510.0f + 510.0f) / 510.0f;  // ≈ 3.96
static const uint16_t ADC_RESOLUTION_BITS = 12;
static const uint16_t ADC_MAX             = (1 << ADC_RESOLUTION_BITS) - 1;

// LiPo discharge curve (rough linear mapping is enough for a % indicator).
static const float    VBAT_FULL_V         = 4.20f;
static const float    VBAT_EMPTY_V        = 3.30f;

// ---------------------------------------------------------------------------
//  BLE UUIDs (must match Flutter app exactly — do not edit)
// ---------------------------------------------------------------------------
#define BLE_SERVICE_UUID         "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_CHAR_SENSOR_UUID     "19B10001-E8F2-537E-4F6C-D104768A1214"  // NOTIFY, 22 B (v3)
#define BLE_CHAR_BATTERY_UUID    "19B10002-E8F2-537E-4F6C-D104768A1214"  // READ,    1 B
#define BLE_CHAR_CONFIG_UUID     "19B10003-E8F2-537E-4F6C-D104768A1214"  // WRITE,   1 B

// Sensor packet (v3): 22 bytes, little-endian. Firmware always emits v3.
//  [0..1]   uint16  timestamp_ms (relative to session start, wraps at ~65 s)
//  [2..3]   int16   qw * QUAT_SCALE
//  [4..5]   int16   qx * QUAT_SCALE
//  [6..7]   int16   qy * QUAT_SCALE
//  [8..9]   int16   qz * QUAT_SCALE
//  [10..11] int16   accel_x (milli-g)
//  [12..13] int16   accel_y (milli-g)
//  [14..15] int16   accel_z (milli-g)
//  [16..17] int16   gyro_x  * GYRO_SCALE  (°/s)
//  [18..19] int16   gyro_y  * GYRO_SCALE  (°/s)
//  [20..21] int16   gyro_z  * GYRO_SCALE  (°/s)
//
// Legacy formats (parser-side only; the firmware no longer emits them):
//   v1 = 14 B (no accel_z, no gyro)
//   v2 = 16 B (no gyro)
// Clients must degrade by bytes.length and never throw on shorter packets.
static const uint8_t SENSOR_PACKET_SIZE   = 22;

// Quaternion + accel scale factors (mirrored on the app side).
static const float   QUAT_SCALE           = 10000.0f;
static const float   ACCEL_SCALE_MILLI_G  = 1000.0f;   // m/s² → milli-g uses g=9.80665

// Gyroscope scale: int16 = gyro_dps * GYRO_SCALE.
// 100 → range ±327 °/s with 0.01 °/s resolution → covers running and gym.
// For golf (peaks ~2000 °/s on the wrist) drop to 10 — keep firmware and app in sync.
static const float   GYRO_SCALE           = 100.0f;

#endif // CONFIG_H
