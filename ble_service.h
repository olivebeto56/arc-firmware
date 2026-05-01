// ============================================================================
//  ble_service.h — BLE GATT layer (Sprint 1)
//
//  Exposes one custom service with three characteristics:
//    19B10001 — sensor data (NOTIFY,  22 B, packet v3 — quat + linear accel + gyro)
//    19B10002 — battery %   (READ,     1 B)
//    19B10003 — config      (WRITE,    1 B)  reserved for future sample-rate change
//
//  Behaviour on disconnect: re-start advertising automatically so the phone can
//  reconnect mid-session (timestamp_ms restarts on reconnect — the app handles it).
// ============================================================================
#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <Arduino.h>
#include "sensor.h"

// Bring the BLE stack up and start advertising as BLE_LOCAL_NAME.
// Returns true on success.
bool bleInit();

// Service the BLE stack — call every loop iteration.
void blePoll();

// True while a central is connected. Cheap; safe to call frequently.
bool bleConnected();

// Pack `data` as the 22-byte v3 packet and notify subscribers.
// session_start_ms lets us send a 16-bit timestamp relative to session start
// (wraps every ~65 s; the app reconstructs absolute time using packet order).
void bleSendSensor(const SensorData& data, uint32_t session_start_ms);

// Update the value behind the battery characteristic. Reads happen on demand
// from the central, but this also pushes a notify so subscribers can avoid polling.
void bleUpdateBatteryPercent(uint8_t percent);

#endif // BLE_SERVICE_H
