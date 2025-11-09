// ble_telemetry.cpp
// Complete BLE telemetry module (fixed compilation errors)
#include "ble_telemetry.h"
#include <NimBLEDevice.h>
#include <string.h>
#include <string>
#include <math.h>

// ---------------- CONFIG ----------------
static const uint16_t COMPANY_ID = 0x02E5; // little-endian in adv (E5 02)
static const uint8_t PROTO_VER = 1;
// Friendly device name shown in most scanners (change for your device)
// Friendly device name shown in most scanners (change for your device)
static const char* DEVICE_NAME = "ESP32_BLE";

// UUIDs
static const char* TELEMETRY_SVC_UUID    = "0000A100-0000-1000-8000-00805F9B34FB";
static const char* TELEMETRY_STREAM_UUID = "0000A101-0000-1000-8000-00805F9B34FB";
static const char* CONTROL_UUID          = "0000A103-0000-1000-8000-00805F9B34FB";

// Connectable window (ms) when urgent seen
static const uint32_t CONNECTABLE_WINDOW_MS = 30 * 1000UL; // 30s

// Advert intervals (best-effort)
static const uint32_t ADVERT_INTERVAL_MS_NORMAL = 500;
static const uint32_t ADVERT_INTERVAL_MS_URGENT = 120;

// Require application-level auth for notifications (true = require token before streaming)
// Disable application-level auth for now to allow easy connection during testing
// (commenting out auth logic that may prevent clients from receiving notifications)
static const bool REQUIRE_APP_AUTH = false;

// Pre-shared 32-bit token (example). Change to your own secret for your client.
static const uint32_t APP_AUTH_TOKEN = 0xA1B2C3D4; // client must write this (LE byte order) to CONTROL char

// ---------------- internal state ----------------
static Telemetry lastSnapshot;
static SemaphoreHandle_t snapshotMutex = nullptr;

static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pStreamChar = nullptr;
static NimBLECharacteristic* pControlChar = nullptr;
static NimBLEAdvertising* pAdvertising = nullptr;

static bool deviceConnected = false;
static bool clientAuthorized = false; // set true when client writes correct token
static uint8_t seqNo = 0;

static uint32_t connectableUntil = 0; // ms timestamp until which advertising is connectable
static bool isCurrentlyConnectable = false;

// store last manufacturer data blob so we can reapply it (NimBLEAdvertising has no getter)
static std::string lastManufacturerData;

// lightweight ACK + retransmit variables
static uint8_t lastPacketBuf[64];
static uint8_t lastPacketLen = 0;
static uint32_t lastSentAt = 0;
static const uint32_t RETRANSMIT_TIMEOUT_MS = 1000; // resend if no ack within 1s
static bool awaitingAck = false;

// Startup snapshot capture + manufacturer update throttling
// For testing set to 0 so initial manufacturer blob is captured immediately after BLE setup.
// Set back to 4000 (or a few seconds) for production to let sensors stabilize.
static const uint32_t STARTUP_SNAPSHOT_DELAY_MS = 0; // was 4000
static const uint32_t MANUF_UPDATE_INTERVAL_MS = 10000; // after initial snapshot, update manuf data at most every 10s
static uint32_t startupCaptureUntil = 0;
static bool startupSnapshotCaptured = false;
static uint32_t lastManufacturerUpdateAt = 0;

// For quick testing: set to true to force the device to advertise as CONNECTABLE on startup.
// This makes the device show up in phone/OS Bluetooth lists that only display connectable devices.
// Remember to set back to 'false' for production to preserve non-connectable low-power behaviour.
static const bool FORCE_CONNECTABLE_FOR_TESTING = true;

// Helper: print bytes as hex to Serial
static void printHex(const std::string &s) {
  for (size_t i = 0; i < s.size(); ++i) {
    uint8_t b = (uint8_t)s[i];
    if (b < 16) Serial.print('0');
    Serial.print(b, HEX);
    if (i + 1 < s.size()) Serial.print(' ');
  }
  Serial.println();
}

// ---------- CRC8/MAXIM ----------
static uint8_t crc8_maxim(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    uint8_t in = data[i];
    crc ^= in;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x01) crc = (crc >> 1) ^ 0x8C;
      else crc = (crc >> 1);
    }
  }
  return crc;
}

// Utility write int24 LE
static void writeInt24LE(uint8_t* out, int32_t v) {
  out[0] = v & 0xFF;
  out[1] = (v >> 8) & 0xFF;
  out[2] = (v >> 16) & 0xFF;
}

// Convenience: millis wrapper
static inline uint32_t now_ms() { return (uint32_t)millis(); }

// Forward declarations (with default arguments)
static void buildAndSetAdvertFromSnapshot();
static void sendNotificationIfConnected();
static void evaluateAlertsFromSnapshot(Telemetry &s);
static void requestConnectableWindow(uint32_t ms);
static void startNonConnectableAdvert(uint32_t intervalMs = ADVERT_INTERVAL_MS_NORMAL);
static void startConnectableAdvert(uint32_t intervalMs = ADVERT_INTERVAL_MS_URGENT);
static void handleAck(uint8_t ackSeq); // ADDED: Forward declaration for handleAck

// ---------------- NimBLE Server callbacks ----------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server) {
    deviceConnected = true;
    // For testing: disable re-auth requirement so clients can connect immediately
    clientAuthorized = true;
    Serial.println("[BLE] Client connected (auth disabled for testing)");
  }
  void onDisconnect(NimBLEServer* server) {
    deviceConnected = false;
    clientAuthorized = false;
    Serial.println("[BLE] Client disconnected");
    // Restart advertising (task loop will ensure correct mode)
    if (pAdvertising) pAdvertising->start();
  }
};

// ---------------- Control char write callback ----------------
class ControlCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr) {
    std::string v = chr->getValue();

    // check for 3-byte ACK: "ACK" + seq
    if (v.size() == 4 && v[0]=='A' && v[1]=='C' && v[2]=='K') {
      handleAck((uint8_t)v[3]);
      return;
    }

    if (v.size() == 4) {
      // interpret 4 bytes as little-endian uint32 token
      uint32_t tok = (uint8_t)v[0] | ((uint8_t)v[1]<<8) | ((uint8_t)v[2]<<16) | ((uint8_t)v[3]<<24);
      if (tok == APP_AUTH_TOKEN) {
        clientAuthorized = true;
        Serial.println("[BLE] Client authorized via token");
        // Optionally: reply with ACK via same char
        const char ack[] = "OK";
        chr->setValue((uint8_t*)ack, 2);
      } else {
        clientAuthorized = false;
        Serial.println("[BLE] Client auth token invalid");
        const char nak[] = "NO";
        chr->setValue((uint8_t*)nak, 2);
      }
    } else {
      // Could be other control commands. For now we ignore/ack unknown lengths.
      Serial.printf("[BLE] Control write len=%u\n", (unsigned)v.size());
    }
  }
};

// ---------------- Alert smoothing constants (same as before) ----------------
static const int CONSECUTIVE_FOR_ABNORMAL = 3;
static const int CONSECUTIVE_FOR_GPS_LOSS = 3;
static const int HOLD_MS_IMU_EVENTS = 3000;

static int lowBpmCount = 0, highBpmCount = 0, lowSpO2Count = 0, co2HighCount = 0;
static int gpsGoodCount = 0, gpsBadCount = 0;
static uint32_t lastFreefallSeenAt = 0, lastVibrationSeenAt = 0, lastShockSeenAt = 0, lastFallSeenAt = 0, lastTiltSeenAt = 0;

// ---------------- Evaluate alerts (same algorithm from earlier) ----------------
static void evaluateAlertsFromSnapshot(Telemetry &s) {
  uint32_t now = now_ms();
  if (s.freefall) lastFreefallSeenAt = now;
  if (s.vibration) lastVibrationSeenAt = now;
  if (s.shock) lastShockSeenAt = now;
  if (s.fall) lastFallSeenAt = now;
  if (s.tilt) lastTiltSeenAt = now;

  bool heldFreefall = (lastFreefallSeenAt != 0) && ((int32_t)(now - lastFreefallSeenAt) < HOLD_MS_IMU_EVENTS);
  bool heldVibration = (lastVibrationSeenAt != 0) && ((int32_t)(now - lastVibrationSeenAt) < HOLD_MS_IMU_EVENTS);
  bool heldShock = (lastShockSeenAt != 0) && ((int32_t)(now - lastShockSeenAt) < HOLD_MS_IMU_EVENTS);
  bool heldFall = (lastFallSeenAt != 0) && ((int32_t)(now - lastFallSeenAt) < HOLD_MS_IMU_EVENTS);
  bool heldTilt = (lastTiltSeenAt != 0) && ((int32_t)(now - lastTiltSeenAt) < HOLD_MS_IMU_EVENTS);

  s.freefall = heldFreefall;
  s.vibration = heldVibration;
  s.shock = heldShock;
  s.fall = heldFall;
  s.tilt = heldTilt;

  // BPM/SpO2 logic
  float bpm = s.bpm;
  float spo2 = s.spo2;
  bool bpmValid = (bpm > 0.5f && isfinite(bpm));
  bool spo2Valid = (spo2 > 0.5f && isfinite(spo2));

  const float BPM_LOW = 60.0f;
  const float BPM_HIGH = 100.0f;
  const float SPO2_LOW = 90.0f;

  if (bpmValid && (bpm < BPM_LOW)) lowBpmCount++; else lowBpmCount = 0;
  if (bpmValid && (bpm > BPM_HIGH)) highBpmCount++; else highBpmCount = 0;
  if (spo2Valid && (spo2 < SPO2_LOW)) lowSpO2Count++; else lowSpO2Count = 0;
  float co2 = s.co2ppm;
  if (isfinite(co2) && co2 > 1000.0f) co2HighCount++; else co2HighCount = 0;

  bool abnormalVitals = (lowBpmCount >= CONSECUTIVE_FOR_ABNORMAL) ||
                        (highBpmCount >= CONSECUTIVE_FOR_ABNORMAL) ||
                        (lowSpO2Count >= CONSECUTIVE_FOR_ABNORMAL) ||
                        (co2HighCount >= CONSECUTIVE_FOR_ABNORMAL);

  // GPS debouncing
  bool gpsRawOK = s.gpsOK && isfinite(s.lat) && isfinite(s.lon) && (fabs(s.lat) > 1e-7 || fabs(s.lon) > 1e-7);
  if (gpsRawOK) { gpsGoodCount++; gpsBadCount = 0; } else { gpsBadCount++; gpsGoodCount = 0; }
  bool gpsValid = (gpsGoodCount >= 1);
  if (gpsBadCount >= CONSECUTIVE_FOR_GPS_LOSS) gpsValid = false;

  s.gpsOK = gpsValid;

  // encode abnormal into vibration flag as lightweight indicator (keeps struct unchanged)
  if (abnormalVitals) s.vibration = true;
}

// ---------------- Advertising mode helpers ----------------
// Request connectable window (ms). call when urgent event seen.
static void requestConnectableWindow(uint32_t ms) {
  uint32_t t = now_ms();
  // avoid overflow: if connectableUntil already large, extend
  if (connectableUntil == 0 || t + ms > connectableUntil) connectableUntil = t + ms;
  Serial.printf("[BLE] Requested connectable window %u ms (until %lu)\n", (unsigned)ms, (unsigned long)connectableUntil);
}

// Attempt to start non-connectable advertising.
// Use NimBLEAdvertisingParams if available; fallback otherwise.
// FIXED: Removed default argument (already in forward declaration)
static void startNonConnectableAdvert(uint32_t intervalMs) {
  if (!pAdvertising) return;
  Serial.println("[BLE] Starting NON-CONNECTABLE advert");
#if defined(NimBLEAdvertisingParams)
  NimBLEAdvertisingParams params;
  params.setConnectable(false);
  // leave interval as default (implementation-specific) or set if available
  pAdvertising->stop();
  if (!lastManufacturerData.empty()) {
    Serial.print("[BLE] Setting manufacturer data for non-connectable advert: ");
    printHex(lastManufacturerData);
    pAdvertising->setManufacturerData(lastManufacturerData);
  }
  pAdvertising->start(params);
#else
  // fallback: restart advertising with lastManufacturerData (best-effort)
  pAdvertising->stop();
  if (!lastManufacturerData.empty()) {
    pAdvertising->setManufacturerData(lastManufacturerData);
  }
  pAdvertising->start();
#endif
  isCurrentlyConnectable = false;
}

// Attempt to start connectable advertising
// FIXED: Removed default argument (already in forward declaration)
static void startConnectableAdvert(uint32_t intervalMs) {
  if (!pAdvertising) return;
  Serial.println("[BLE] Starting CONNECTABLE advert");
#if defined(NimBLEAdvertisingParams)
  NimBLEAdvertisingParams params;
  params.setConnectable(true);
  pAdvertising->stop();
  if (!lastManufacturerData.empty()) {
    Serial.print("[BLE] Setting manufacturer data for connectable advert: ");
    printHex(lastManufacturerData);
    pAdvertising->setManufacturerData(lastManufacturerData);
  }
  pAdvertising->start(params);
#else
  // fallback: ensure service UUID present (makes client discovery easier) and restart
  pAdvertising->stop();
  // service UUID will be provided in the scan response to avoid adv payload overflow
  if (!lastManufacturerData.empty()) {
    pAdvertising->setManufacturerData(lastManufacturerData);
  }
  pAdvertising->start();
#endif
  isCurrentlyConnectable = true;
}

// ---------------- Build advert from snapshot ----------------
// manufacturer data layout (21 bytes): see preceding docs
static void buildAndSetAdvertFromSnapshot() {
  Telemetry s;
  if (snapshotMutex && xSemaphoreTake(snapshotMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    s = lastSnapshot;
    xSemaphoreGive(snapshotMutex);
  } else {
    // default zero snapshot
  }

  // stabilize flags
  evaluateAlertsFromSnapshot(s);

  // Build manufacturer payload
  uint8_t buf[21];
  memset(buf, 0, sizeof(buf));
  buf[0] = COMPANY_ID & 0xFF;
  buf[1] = (COMPANY_ID >> 8) & 0xFF;
  buf[2] = PROTO_VER;
  uint8_t flags = 0;
  if (s.freefall) flags |= (1<<0);
  if (s.vibration) flags |= (1<<1);
  if (s.tilt) flags |= (1<<2);
  bool urgent = s.fall || s.freefall ||
                ((s.bpm > 0.5f && (s.bpm < 60.0f || s.bpm > 100.0f)) ||
                 (s.spo2 > 0.5f && s.spo2 < 90.0f));
  if (urgent) flags |= (1<<3);
  if (s.vibration) flags |= (1<<5); // abnormalVitals proxy
  if (s.gpsOK) flags |= (1<<6);
  buf[3] = flags;

  buf[4] = 100; // battery placeholder
  buf[5] = (uint8_t)((s.bpm > 0.5f && isfinite(s.bpm)) ? constrain((int)round(s.bpm), 0, 255) : 0);
  buf[6] = (uint8_t)((s.spo2 > 0.5f && isfinite(s.spo2)) ? constrain((int)round(s.spo2), 0, 255) : 0);
  uint16_t co2 = (uint16_t)constrain((int)round(isnan(s.co2ppm) ? 0 : s.co2ppm), 0, 65535);
  buf[7] = co2 & 0xFF; buf[8] = (co2>>8) & 0xFF;
  int16_t temp_centi = (int16_t)round(isnan(s.tempBME) ? 0 : (s.tempBME * 100.0f));
  buf[9] = temp_centi & 0xFF; buf[10] = (temp_centi >> 8) & 0xFF;

  int32_t lat_e5 = (int32_t)round(s.lat * 1e5);
  int32_t lon_e5 = (int32_t)round(s.lon * 1e5);
  writeInt24LE(&buf[11], lat_e5);
  writeInt24LE(&buf[14], lon_e5);

  uint32_t ts = (uint32_t)((millis()/1000) & 0xFFFFFF);
  buf[17] = ts & 0xFF; buf[18] = (ts>>8) & 0xFF; buf[19] = (ts>>16) & 0xFF;

  uint8_t crc = crc8_maxim(buf, 20);
  std::string man;
  man.resize(21);
  memcpy(&man[0], buf, 20);
  man[20] = crc;
  uint32_t now = now_ms();

  // If we're still in the startup capture window, wait until the window expires
  // and take one stable snapshot to use as the manufacturer data.
  if (!startupSnapshotCaptured) {
    if (now < startupCaptureUntil) {
      // skip updating manuf data for now (stabilize sensors)
      return;
    }
    // capture initial stable manufacturer blob
    lastManufacturerData = man;
    startupSnapshotCaptured = true;
    lastManufacturerUpdateAt = now;
    Serial.println("[BLE] Startup manufacturer snapshot captured");
    Serial.print("[BLE] Initial manufacturer data: "); printHex(lastManufacturerData);
    if (pAdvertising) {
      // Put Name + 128-bit Service UUID into the advertising packet (fits within 31 bytes)
      NimBLEAdvertisementData advData;
      advData.setName(DEVICE_NAME);
      advData.addServiceUUID(TELEMETRY_SVC_UUID);
      pAdvertising->setAdvertisementData(advData);
      // Put Manufacturer data into the scan response so it fits there
      NimBLEAdvertisementData scanResp;
      scanResp.setManufacturerData(lastManufacturerData);
      pAdvertising->setScanResponseData(scanResp);
      pAdvertising->start();
    }
    // If urgent: request connectable window
    if (urgent) requestConnectableWindow(CONNECTABLE_WINDOW_MS);
    return;
  }

  // After initial capture, throttle manufacturer updates to avoid rapid churn
  if ((now - lastManufacturerUpdateAt) < MANUF_UPDATE_INTERVAL_MS) {
    // too soon to update
    return;
  }

  // Time to update the manufacturer payload
  lastManufacturerData = man;
  lastManufacturerUpdateAt = now;
  if (pAdvertising) {
    NimBLEAdvertisementData advData;
    // Put Name + 128-bit Service UUID into the advertising packet (fits within 31 bytes)
    advData.setName(DEVICE_NAME);
    advData.addServiceUUID(TELEMETRY_SVC_UUID);
    pAdvertising->setAdvertisementData(advData);
    NimBLEAdvertisementData scanResp;
    // Put Manufacturer data into the scan response so scanners will show it
    scanResp.setManufacturerData(lastManufacturerData);
    pAdvertising->setScanResponseData(scanResp);
    Serial.print("[BLE] Updated manufacturer data: ");
    printHex(lastManufacturerData);
    if (urgent) requestConnectableWindow(CONNECTABLE_WINDOW_MS);
    pAdvertising->start();
  }
}

// ---------------- Notifications ----------------
static void sendNotificationIfConnected() {
  if (!deviceConnected || !pStreamChar) return;
  if (REQUIRE_APP_AUTH && !clientAuthorized) {
    // Do not send full notifications until client authorized
    return;
  }
  Telemetry s;
  if (snapshotMutex && xSemaphoreTake(snapshotMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    s = lastSnapshot;
    xSemaphoreGive(snapshotMutex);
  }

  // ensure snapshot and adv flags consistent
  evaluateAlertsFromSnapshot(s);

  // Build tiny stream packet: header(2), type, seq, ts6, bpm, spo2, co2, flags
  uint8_t packet[1+1+1+1+6+1+1+2+1];
  int idx = 0;
  packet[idx++] = 0xA1; packet[idx++] = 0xB2;
  packet[idx++] = 1; // type = stream
  packet[idx++] = seqNo++;
  uint64_t ts = (uint64_t)millis();
  for (int i = 0; i < 6; ++i) packet[idx++] = (ts >> (8*i)) & 0xFF;
  packet[idx++] = (uint8_t)((s.bpm > 0.5f && isfinite(s.bpm)) ? constrain((int)round(s.bpm), 0, 255) : 0);
  packet[idx++] = (uint8_t)((s.spo2 > 0.5f && isfinite(s.spo2)) ? constrain((int)round(s.spo2), 0, 255) : 0);
  uint16_t co2v = (uint16_t)constrain((int)round(isnan(s.co2ppm)?0:s.co2ppm), 0, 65535);
  packet[idx++] = co2v & 0xFF; packet[idx++] = (co2v>>8) & 0xFF;
  uint8_t flags = 0;
  if (s.freefall) flags |= (1<<0);
  if (s.vibration) flags |= (1<<1);
  if (s.tilt) flags |= (1<<2);
  if (s.fall) flags |= (1<<3);
  packet[idx++] = flags;

  memcpy(lastPacketBuf, packet, idx);
  // send notify - use indicate for critical one-shot messages if you want guaranteed delivery
  lastPacketLen = idx;
  pStreamChar->notify(lastPacketBuf, lastPacketLen);
  lastSentAt = now_ms(); 
  awaitingAck = true;
}

// ---------------- BLE background task ----------------
static void bleTask(void* pv) {
  const TickType_t advertPeriod = pdMS_TO_TICKS(ADVERT_INTERVAL_MS_NORMAL);
  const TickType_t notifyPeriod = pdMS_TO_TICKS(1000);
  TickType_t lastAdv = xTaskGetTickCount();
  TickType_t lastNotify = xTaskGetTickCount();

  // initial mode: non-connectable
  if (FORCE_CONNECTABLE_FOR_TESTING) {
    Serial.println("[BLE] FORCING CONNECTABLE advert for testing (set FORCE_CONNECTABLE_FOR_TESTING=false to revert)");
    startConnectableAdvert(ADVERT_INTERVAL_MS_NORMAL);
  } else {
    startNonConnectableAdvert(ADVERT_INTERVAL_MS_NORMAL);
  }

  while (true) {
    TickType_t nowT = xTaskGetTickCount();
    uint32_t now = now_ms();

    // decide advertising connectability based on connectableUntil
    if (connectableUntil != 0 && now < connectableUntil) {
      // should be connectable
      if (!isCurrentlyConnectable) {
        startConnectableAdvert(ADVERT_INTERVAL_MS_URGENT);
      }
    } else {
      // ensure non-connectable mode
      if (isCurrentlyConnectable) {
        startNonConnectableAdvert(ADVERT_INTERVAL_MS_NORMAL);
      }
      // clear connectableUntil when expired
      if (connectableUntil != 0 && now >= connectableUntil) {
        connectableUntil = 0;
      }
    }

    if (nowT - lastAdv >= advertPeriod) {
      lastAdv = nowT;
      // refresh advert manufacturer payload (reads lastSnapshot)
      buildAndSetAdvertFromSnapshot();
    }

    if (deviceConnected && (nowT - lastNotify >= notifyPeriod)) {
      lastNotify = nowT;
      sendNotificationIfConnected();
    }

    // resend if ACK not received in time
    if (awaitingAck && (now - lastSentAt > RETRANSMIT_TIMEOUT_MS)) {
        Serial.printf("[BLE] Retransmitting seq %u\n", (uint8_t)(seqNo - 1));
        pStreamChar->notify(lastPacketBuf, lastPacketLen);
        lastSentAt = now;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------------- Public API ----------------
void ble_setup() {
  if (!snapshotMutex) snapshotMutex = xSemaphoreCreateMutex();

  Serial.println("[BLE] Initializing NimBLE");
  // Use a configurable device name so it's easy to spot in scanners like nRF Connect
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // Optional: enable pairing & bonding at NimBLE level (commented)
  // NimBLEDevice::initSecurity();
  // NimBLEDevice::setSecurityAuth(...); // depends on NimBLE-Arduino version and your policy

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Print identifying info to Serial so user can match in nRF Connect without connecting
  if (pServer) {
    Serial.printf("[BLE] Server created. Device name: %s\n", DEVICE_NAME);
  } else {
    Serial.println("[BLE] ERROR: failed to create NimBLE server");
  }
  // Print BLE address if available
  auto addr = NimBLEDevice::getAddress();
  Serial.printf("[BLE] Address: %s\n", addr.toString().c_str());

  NimBLEService* svc = pServer->createService(TELEMETRY_SVC_UUID);

  pStreamChar = svc->createCharacteristic(
    TELEMETRY_STREAM_UUID,
    NIMBLE_PROPERTY::NOTIFY // we'll use notify; server enforces app-level auth
  );

  // Control char: write for token/commands
  pControlChar = svc->createCharacteristic(
    CONTROL_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ
  );
  pControlChar->setCallbacks(new ControlCallback());

  svc->start();

  pAdvertising = NimBLEDevice::getAdvertising();
  // Note: service UUID will be placed in scan response to avoid adv payload overflow
  // Also set the advertising name field explicitly so scanners display the local name
    // We'll set the Local Name explicitly in the advertisement payload (advData.setName)
    // Avoid calling pAdvertising->setName() to prevent duplicate name fields in adv+scan response.

  Serial.print("[BLE] Service UUID: "); Serial.println(TELEMETRY_SVC_UUID);
  Serial.print("[BLE] Stream Char UUID: "); Serial.println(TELEMETRY_STREAM_UUID);
  Serial.print("[BLE] Control  Char UUID: "); Serial.println(CONTROL_UUID);

  // (Note: not all NimBLE builds expose setScanResponse; manufacturer data is set explicitly later)

  // initial advert payload empty until ble_publish_snapshot called
  // schedule startup snapshot capture
  startupCaptureUntil = now_ms() + STARTUP_SNAPSHOT_DELAY_MS;
  startupSnapshotCaptured = false;
  lastManufacturerUpdateAt = 0;
  buildAndSetAdvertFromSnapshot();

  // Print current manufacturer data (if any)
  if (!lastManufacturerData.empty()) {
    Serial.print("[BLE] Initial manufacturer data: ");
    printHex(lastManufacturerData);
  } else {
    Serial.println("[BLE] Initial manufacturer data: <empty>");
  }
}

void ble_start() {
  if (pAdvertising) {
    pAdvertising->start();
    Serial.println("[BLE] Advertising started");
  }
  // Create background BLE task pinned to core 0 (avoid interfering with sensor tasks)
  xTaskCreatePinnedToCore(bleTask, "BLETask", 8192, NULL, 5, NULL, 0);
}

// Publish snapshot (called by main/OLED task). Thread-safe copy.
void ble_publish_snapshot(const Telemetry* snapshot) {
  if (!snapshot) return;
  if (!snapshotMutex) snapshotMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(snapshotMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    lastSnapshot = *snapshot;
    xSemaphoreGive(snapshotMutex);
  }
  // Immediately update advert (kick)
  buildAndSetAdvertFromSnapshot();
}

// called whenever the client sends "ACK <seq>"
static void handleAck(uint8_t ackSeq) {
  if (awaitingAck && ackSeq == (uint8_t)(seqNo - 1)) {
    awaitingAck = false;
    Serial.printf("[BLE] ACK received for seq %u\n", ackSeq);
  }
}