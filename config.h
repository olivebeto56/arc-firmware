// ============================================================================
//  config.h — AI Sport Monitor / running node
//  XIAO nRF52840 Sense + BNO085 + LiPo 400mAh
//
//  Shared configuration: BLE UUIDs, pin map, sample rate, node identity.
//  Change NODE_SIDE between LEFT and RIGHT before flashing each unit.
// ============================================================================
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
//  Node identity  ── change this single line per physical node, then reflash
// ---------------------------------------------------------------------------
#define NODE_SIDE_LEFT   0
#define NODE_SIDE_RIGHT  1

#ifndef NODE_SIDE
  #define NODE_SIDE  NODE_SIDE_LEFT     // <-- set to NODE_SIDE_RIGHT for the other ankle
#endif

#if NODE_SIDE == NODE_SIDE_LEFT
  #define NODE_ID         "LEFT_ANKLE"
  #define BLE_LOCAL_NAME  "SportBand-L"
#elif NODE_SIDE == NODE_SIDE_RIGHT
  #define NODE_ID         "RIGHT_ANKLE"
  #define BLE_LOCAL_NAME  "SportBand-R"
#else
  #error "NODE_SIDE must be NODE_SIDE_LEFT or NODE_SIDE_RIGHT"
#endif

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
static const uint8_t  BATTERY_ENABLE_PIN  = 14;       // P0.14, LOW = divider ON
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
#define BLE_CHAR_SENSOR_UUID     "19B10001-E8F2-537E-4F6C-D104768A1214"  // NOTIFY, 16 B
#define BLE_CHAR_BATTERY_UUID    "19B10002-E8F2-537E-4F6C-D104768A1214"  // READ,    1 B
#define BLE_CHAR_CONFIG_UUID     "19B10003-E8F2-537E-4F6C-D104768A1214"  // WRITE,   1 B

// Sensor packet (v2): 16 bytes, little-endian.
//  [0..1]   uint16  timestamp_ms (relative to session start, wraps at ~65 s)
//  [2..3]   int16   qw * 10000
//  [4..5]   int16   qx * 10000
//  [6..7]   int16   qy * 10000
//  [8..9]   int16   qz * 10000
//  [10..11] int16   accel_x (milli-g)
//  [12..13] int16   accel_y (milli-g)
//  [14..15] int16   accel_z (milli-g)
static const uint8_t SENSOR_PACKET_SIZE   = 16;

// Quaternion + accel scale factors (mirrored on the app side).
static const float   QUAT_SCALE           = 10000.0f;
static const float   ACCEL_SCALE_MILLI_G  = 1000.0f;   // m/s² → milli-g uses g=9.80665

#endif // CONFIG_H
