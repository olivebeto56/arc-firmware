// ============================================================================
//  sensor.h — Sensor abstraction (Sprint 1)
//
//  Hides the BNO085 driver behind a generic SensorData struct so we can swap
//  to a custom-PCB IMU later by replacing only sensor.cpp.
//  Always uses the BNO085 ARVR-stabilized rotation vector (9-axis fused) plus
//  linear acceleration (gravity removed) — required by the running pipeline.
// ============================================================================
#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

// Generic sample produced by the sensor layer. The BLE layer does NOT know
// which IMU is underneath — it only consumes this struct.
struct SensorData {
  uint32_t timestamp_ms;   // millis() at the moment the sample was read
  // Quaternion (Hamilton convention: w, x, y, z), unitless [-1, 1]
  float qw;
  float qx;
  float qy;
  float qz;
  // Linear acceleration (gravity removed) in m/s², body frame
  float ax;
  float ay;
  float az;
  // Calibrated gyroscope rate in °/s, body frame. BNO085 SH2_GYROSCOPE_CALIBRATED
  // applies on-chip bias correction, so this is bias-free for steady operation.
  float gx;
  float gy;
  float gz;
  bool  valid;             // false if the read failed / no fresh report
};

// Initialize the underlying IMU. Returns true on success.
// Must be called after Wire.begin() / Wire.setClock(I2C_CLOCK_HZ).
bool sensorInit();

// Pull the latest fused sample. Non-blocking: returns valid=false if the
// BNO085 has no new report this tick. Call from the main loop at SAMPLE_RATE_HZ.
SensorData getSensorAngles();

// Diagnostic — prints sensor IDs / firmware version over Serial.
void sensorPrintInfo();

#endif // SENSOR_H
