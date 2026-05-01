// ============================================================================
//  ble_service.cpp — ArduinoBLE implementation
//
//  Library: ArduinoBLE >= 1.3.0 (works on the Seeed XIAO nRF52840 Sense board
//  package). We use the stock ArduinoBLE GATT API; no Bluefruit/Adafruit nRF
//  stack required.
// ============================================================================
#include "ble_service.h"
#include "config.h"

#include <ArduinoBLE.h>

// Service + characteristics. The sensor characteristic carries 16 bytes
// per notify; ArduinoBLE will negotiate MTU automatically and split if needed.
static BLEService               sportService(BLE_SERVICE_UUID);
static BLECharacteristic        sensorChar(
    BLE_CHAR_SENSOR_UUID,
    BLENotify,
    SENSOR_PACKET_SIZE,
    /*fixedLength=*/true);
static BLEByteCharacteristic    batteryChar(
    BLE_CHAR_BATTERY_UUID,
    BLERead | BLENotify);
static BLEByteCharacteristic    configChar(
    BLE_CHAR_CONFIG_UUID,
    BLEWrite);

// Packed buffer used as the source for every notify. Static to avoid stack churn.
static uint8_t sensorPacket[SENSOR_PACKET_SIZE];

// ---------------------------------------------------------------------------
//  Connection callbacks — restart advertising on disconnect
// ---------------------------------------------------------------------------
static void onConnect(BLEDevice central) {
  Serial.print(F("[ble] connected: "));
  Serial.println(central.address());
}

static void onDisconnect(BLEDevice central) {
  Serial.print(F("[ble] disconnected: "));
  Serial.println(central.address());
  Serial.println(F("[ble] re-advertising"));
  BLE.advertise();   // critical: resume advertising so the phone can reconnect mid-session
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------
bool bleInit() {
  if (!BLE.begin()) {
    Serial.println(F("[ble] BLE.begin() failed"));
    return false;
  }

  BLE.setLocalName(BLE_LOCAL_NAME);
  BLE.setDeviceName(BLE_LOCAL_NAME);
  BLE.setAdvertisedService(sportService);

  // Default values
  uint8_t zero[SENSOR_PACKET_SIZE] = {0};
  sensorChar.writeValue(zero, SENSOR_PACKET_SIZE);
  batteryChar.writeValue((uint8_t)0);
  configChar.writeValue((uint8_t)(SAMPLE_RATE_HZ / 10));  // hint only; not used yet

  sportService.addCharacteristic(sensorChar);
  sportService.addCharacteristic(batteryChar);
  sportService.addCharacteristic(configChar);
  BLE.addService(sportService);

  BLE.setEventHandler(BLEConnected,    onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);

  if (!BLE.advertise()) {
    Serial.println(F("[ble] advertise() failed"));
    return false;
  }

  Serial.print(F("[ble] advertising as "));
  Serial.println(BLE_LOCAL_NAME);
  return true;
}

void blePoll() {
  BLE.poll();
}

bool bleConnected() {
  // BLE.central() is non-blocking once a peer is already linked.
  BLEDevice c = BLE.central();
  return c && c.connected();
}

// ---------------------------------------------------------------------------
//  Packet packer — keep this byte-for-byte aligned with CLAUDE.md / Flutter parser
// ---------------------------------------------------------------------------
static inline void put_u16_le(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline void put_i16_le(uint8_t* p, int16_t v) {
  put_u16_le(p, (uint16_t)v);
}

// Saturate float -> int16 to avoid wraparound on extreme values.
static inline int16_t sat_i16(float v) {
  if (v >  32767.0f) return  32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

void bleSendSensor(const SensorData& data, uint32_t session_start_ms) {
  if (!bleConnected()) return;

  // 16-bit relative timestamp (wraps every ~65 s; OK because the app reorders
  // by arrival and uses cadence intervals, not absolute time).
  uint16_t ts = (uint16_t)((data.timestamp_ms - session_start_ms) & 0xFFFF);

  // Quaternion: scale by 10000 and clamp.
  int16_t qw = sat_i16(data.qw * QUAT_SCALE);
  int16_t qx = sat_i16(data.qx * QUAT_SCALE);
  int16_t qy = sat_i16(data.qy * QUAT_SCALE);
  int16_t qz = sat_i16(data.qz * QUAT_SCALE);

  // Acceleration: m/s² -> milli-g (×1000/9.80665).
  const float MS2_TO_MILLIG = ACCEL_SCALE_MILLI_G / 9.80665f;
  int16_t ax = sat_i16(data.ax * MS2_TO_MILLIG);
  int16_t ay = sat_i16(data.ay * MS2_TO_MILLIG);
  int16_t az = sat_i16(data.az * MS2_TO_MILLIG);

  put_u16_le(&sensorPacket[0],  ts);
  put_i16_le(&sensorPacket[2],  qw);
  put_i16_le(&sensorPacket[4],  qx);
  put_i16_le(&sensorPacket[6],  qy);
  put_i16_le(&sensorPacket[8],  qz);
  put_i16_le(&sensorPacket[10], ax);
  put_i16_le(&sensorPacket[12], ay);
  put_i16_le(&sensorPacket[14], az);

  // writeValue on a NOTIFY char queues a notify to subscribed centrals.
  sensorChar.writeValue(sensorPacket, SENSOR_PACKET_SIZE);
}

void bleUpdateBatteryPercent(uint8_t percent) {
  if (percent > 100) percent = 100;
  batteryChar.writeValue(percent);
}
