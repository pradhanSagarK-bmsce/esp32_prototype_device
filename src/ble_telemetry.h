// ble_telemetry.h
#pragma once
#include "telemetry_shared.h"

// Debug flag for BLE operations
#define BLE_DEBUG 1

#ifdef __cplusplus
extern "C" {
#endif

// Initialize BLE stack and create GATT server (call once in setup before start)
void ble_setup();

// Start BLE task which will manage advertising and notifications
void ble_start();

// Copy a snapshot into BLE module for immediate advertise + notify use.
// Call this from any task after copying Telemetry (while holding dataMutex or passing a copy).
void ble_publish_snapshot(const Telemetry* snapshot);

#ifdef __cplusplus
}
#endif
