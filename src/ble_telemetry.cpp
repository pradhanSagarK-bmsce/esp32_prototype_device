// ble_telemetry.cpp
// FIXED: Proper connectable advertising and GATT server setup
#include "ble_telemetry.h"
#include <NimBLEDevice.h>
#include <string.h>
#include <string>
#include <math.h>

// ---------------- CONFIG ----------------
static const uint16_t COMPANY_ID = 0x02E5;
static const uint8_t PROTO_VER = 1;
static const char* DEVICE_NAME = "ESP32_BLE";

// UUIDs
static const char* TELEMETRY_SVC_UUID    = "0000A100-0000-1000-8000-00805F9B34FB";
static const char* TELEMETRY_STREAM_UUID = "0000A101-0000-1000-8000-00805F9B34FB";
static const char* CONTROL_UUID          = "0000A103-0000-1000-8000-00805F9B34FB";

static const uint32_t CONNECTABLE_WINDOW_MS = 30 * 1000UL;
static const uint32_t ADVERT_INTERVAL_MS_NORMAL = 500;
static const uint32_t ADVERT_INTERVAL_MS_URGENT = 120;

// CRITICAL: Keep connectable for testing
static const bool FORCE_CONNECTABLE_FOR_TESTING = true;
static const bool REQUIRE_APP_AUTH = false;
static const uint32_t APP_AUTH_TOKEN = 0xA1B2C3D4;

// ---------------- internal state ----------------
static Telemetry lastSnapshot;
static SemaphoreHandle_t snapshotMutex = nullptr;

static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pStreamChar = nullptr;
static NimBLECharacteristic* pControlChar = nullptr;
static NimBLEAdvertising* pAdvertising = nullptr;

static bool deviceConnected = false;
static bool clientAuthorized = false;
static bool clientSubscribed = false; // Track subscription state
static uint8_t seqNo = 0;

static uint32_t connectableUntil = 0;
static bool isCurrentlyConnectable = false;
static std::string lastManufacturerData;

static uint8_t lastPacketBuf[64];
static uint8_t lastPacketLen = 0;
static uint32_t lastSentAt = 0;
static const uint32_t RETRANSMIT_TIMEOUT_MS = 1000;
static bool awaitingAck = false;

static const uint32_t STARTUP_SNAPSHOT_DELAY_MS = 0;
static const uint32_t MANUF_UPDATE_INTERVAL_MS = 10000;
static uint32_t startupCaptureUntil = 0;
static bool startupSnapshotCaptured = false;
static uint32_t lastManufacturerUpdateAt = 0;

// Alert smoothing
static const int CONSECUTIVE_FOR_ABNORMAL = 3;
static const int CONSECUTIVE_FOR_GPS_LOSS = 3;
static const int HOLD_MS_IMU_EVENTS = 3000;

static int lowBpmCount = 0, highBpmCount = 0, lowSpO2Count = 0, co2HighCount = 0;
static int gpsGoodCount = 0, gpsBadCount = 0;
static uint32_t lastFreefallSeenAt = 0, lastVibrationSeenAt = 0, lastShockSeenAt = 0, lastFallSeenAt = 0, lastTiltSeenAt = 0;

static void printHex(const std::string &s) {
  for (size_t i = 0; i < s.size(); ++i) {
    uint8_t b = (uint8_t)s[i];
    if (b < 16) Serial.print('0');
    Serial.print(b, HEX);
    if (i + 1 < s.size()) Serial.print(' ');
  }
  Serial.println();
}

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

static void writeInt24LE(uint8_t* out, int32_t v) {
  out[0] = v & 0xFF;
  out[1] = (v >> 8) & 0xFF;
  out[2] = (v >> 16) & 0xFF;
}

static inline uint32_t now_ms() { return (uint32_t)millis(); }

// Forward declarations
static void buildAndSetAdvertFromSnapshot();
static void sendNotificationIfConnected();
static void evaluateAlertsFromSnapshot(Telemetry &s);
static void requestConnectableWindow(uint32_t ms);
static void startAdvertising();
static void handleAck(uint8_t ackSeq);

// ---------------- NimBLE Server callbacks ----------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, ble_gap_conn_desc* desc) {
    deviceConnected = true;
    clientAuthorized = true; // Auto-auth for testing
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║     ✅ CLIENT CONNECTED TO ESP32      ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.printf("[BLE] Device Name: %s\n", DEVICE_NAME);
    Serial.printf("[BLE] Client Address: %s\n", NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    Serial.printf("[BLE] Connection ID: %d\n", desc->conn_handle);
    Serial.printf("[BLE] Status: connected=%d auth=%d\n", deviceConnected, clientAuthorized);
    Serial.println("════════════════════════════════════════\n");
    
    // Update connection parameters for better throughput
    server->updateConnParams(desc->conn_handle, 24, 48, 0, 400);
  }
  
  void onDisconnect(NimBLEServer* server) {
    deviceConnected = false;
    clientAuthorized = false;
    clientSubscribed = false;
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║        ❌ CLIENT DISCONNECTED          ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    // Restart advertising
    delay(500);
    if (pAdvertising) {
      startAdvertising();
    }
  }
};

// CRITICAL: Track subscription state
class StreamCharCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* pChar, ble_gap_conn_desc* desc, uint16_t subValue) {
    if (subValue == 0x0001) {
      clientSubscribed = true;
      Serial.println("\n╔════════════════════════════════════════╗");
      Serial.println("║   🔔 CLIENT SUBSCRIBED TO STREAM!     ║");
      Serial.println("╚════════════════════════════════════════╝");
      Serial.println("[BLE] Notifications ENABLED");
      Serial.println("[BLE] Starting telemetry stream...");
      Serial.println("════════════════════════════════════════\n");
    } else if (subValue == 0x0002) {
      clientSubscribed = true;
      Serial.println("[BLE] 📬 Client enabled INDICATIONS");
    } else if (subValue == 0x0000) {
      clientSubscribed = false;
      Serial.println("[BLE] 🔕 Client UNSUBSCRIBED from notifications");
    }
  }
  
  void onRead(NimBLECharacteristic* pChar) {
    Serial.println("[BLE] 📖 Client READ stream characteristic");
  }
};

// ---------------- Control char write callback ----------------
class ControlCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr) {
    std::string v = chr->getValue();
    
    Serial.printf("[BLE] 📝 Control write received: %d bytes\n", v.size());

    // Check for ACK
    if (v.size() == 4 && v[0]=='A' && v[1]=='C' && v[2]=='K') {
      handleAck((uint8_t)v[3]);
      return;
    }

    if (v.size() == 4) {
      uint32_t tok = (uint8_t)v[0] | ((uint8_t)v[1]<<8) | ((uint8_t)v[2]<<16) | ((uint8_t)v[3]<<24);
      Serial.printf("[BLE] Auth token received: 0x%08X\n", tok);
      if (tok == APP_AUTH_TOKEN) {
        clientAuthorized = true;
        Serial.println("[BLE] 🔐 Client AUTHORIZED via token");
        const char ack[] = "OK";
        chr->setValue((uint8_t*)ack, 2);
        chr->notify();
      } else {
        clientAuthorized = false;
        Serial.println("[BLE] ⚠️ Client auth token INVALID");
        const char nak[] = "NO";
        chr->setValue((uint8_t*)nak, 2);
        chr->notify();
      }
    } else {
      Serial.printf("[BLE] Unknown control command, len=%u\n", (unsigned)v.size());
    }
  }
  
  void onRead(NimBLECharacteristic* chr) {
    Serial.println("[BLE] 📖 Client READ control characteristic");
  }
};

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

  bool gpsRawOK = s.gpsOK && isfinite(s.lat) && isfinite(s.lon) && (fabs(s.lat) > 1e-7 || fabs(s.lon) > 1e-7);
  if (gpsRawOK) { gpsGoodCount++; gpsBadCount = 0; } else { gpsBadCount++; gpsGoodCount = 0; }
  bool gpsValid = (gpsGoodCount >= 1);
  if (gpsBadCount >= CONSECUTIVE_FOR_GPS_LOSS) gpsValid = false;

  s.gpsOK = gpsValid;
  if (abnormalVitals) s.vibration = true;
}

static void requestConnectableWindow(uint32_t ms) {
  uint32_t t = now_ms();
  if (connectableUntil == 0 || t + ms > connectableUntil) connectableUntil = t + ms;
}

// FIXED: Simplified advertising - always connectable for testing
static void startAdvertising() {
  if (!pAdvertising) return;
  
  pAdvertising->stop();
  delay(100);
  
  // Always connectable when forced
  if (FORCE_CONNECTABLE_FOR_TESTING) {
    Serial.println("[BLE] 📡 Starting CONNECTABLE advertising (TESTING MODE)");
    isCurrentlyConnectable = true;
  } else {
    Serial.println("[BLE] 📡 Starting advertising");
  }
  
  // Set advertising data
  NimBLEAdvertisementData advData;
  advData.setName(DEVICE_NAME);
  advData.addServiceUUID(TELEMETRY_SVC_UUID);
  pAdvertising->setAdvertisementData(advData);
  
  // Set scan response with manufacturer data
  if (!lastManufacturerData.empty()) {
    NimBLEAdvertisementData scanResp;
    scanResp.setManufacturerData(lastManufacturerData);
    pAdvertising->setScanResponseData(scanResp);
  }
  
  pAdvertising->start();
  Serial.println("[BLE] ✅ Advertising started");
}

static void buildAndSetAdvertFromSnapshot() {
  Telemetry s;
  if (snapshotMutex && xSemaphoreTake(snapshotMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    s = lastSnapshot;
    xSemaphoreGive(snapshotMutex);
  }

  evaluateAlertsFromSnapshot(s);

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
  if (s.vibration) flags |= (1<<5);
  if (s.gpsOK) flags |= (1<<6);
  buf[3] = flags;

  buf[4] = 100;
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

  if (!startupSnapshotCaptured) {
    if (now < startupCaptureUntil) return;
    lastManufacturerData = man;
    startupSnapshotCaptured = true;
    lastManufacturerUpdateAt = now;
    Serial.println("[BLE] 📸 Startup manufacturer snapshot captured");
    startAdvertising();
    if (urgent) requestConnectableWindow(CONNECTABLE_WINDOW_MS);
    return;
  }

  if ((now - lastManufacturerUpdateAt) < MANUF_UPDATE_INTERVAL_MS) return;

  lastManufacturerData = man;
  lastManufacturerUpdateAt = now;
  
  // Only restart advertising if not connected
  if (!deviceConnected && pAdvertising) {
    startAdvertising();
  }
}

// FIXED: Only send if subscribed
static void sendNotificationIfConnected() {
  if (!deviceConnected || !pStreamChar || !clientSubscribed) {
    return;
  }

  if (REQUIRE_APP_AUTH && !clientAuthorized) {
    return;
  }

  Telemetry s;
  if (snapshotMutex && xSemaphoreTake(snapshotMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    s = lastSnapshot;
    xSemaphoreGive(snapshotMutex);
  }

  evaluateAlertsFromSnapshot(s);

  // Build packet
  uint8_t packet[18];
  int idx = 0;
  packet[idx++] = 0xA1; packet[idx++] = 0xB2;
  packet[idx++] = 1; // type
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
  lastPacketLen = idx;

  // Send notification
  bool sent = pStreamChar->notify(lastPacketBuf, lastPacketLen);
  
  if (sent) {
    Serial.printf("[BLE] 📡 Notification #%d sent: BPM=%d SpO2=%d CO2=%d\n", 
                  (int)(seqNo-1), (int)s.bpm, (int)s.spo2, (int)s.co2ppm);
  } else {
    Serial.println("[BLE] ❌ Notification FAILED to send");
  }
  
  lastSentAt = now_ms();
  awaitingAck = true;
}

// ---------------- BLE background task ----------------
static void bleTask(void* pv) {
  const TickType_t advertPeriod = pdMS_TO_TICKS(5000); // Update adv less frequently
  const TickType_t notifyPeriod = pdMS_TO_TICKS(1000);
  TickType_t lastAdv = xTaskGetTickCount();
  TickType_t lastNotify = xTaskGetTickCount();

  Serial.println("[BLE] 🚀 BLE Task started");

  while (true) {
    TickType_t nowT = xTaskGetTickCount();
    uint32_t now = now_ms();

    // Update advertising data periodically (but don't restart if connected)
    if (!deviceConnected && (nowT - lastAdv >= advertPeriod)) {
      lastAdv = nowT;
      buildAndSetAdvertFromSnapshot();
    }

    // Send notifications if connected and subscribed
    if (deviceConnected && clientSubscribed && (nowT - lastNotify >= notifyPeriod)) {
      lastNotify = nowT;
      sendNotificationIfConnected();
    }

    // Retransmit if needed
    if (awaitingAck && clientSubscribed && (now - lastSentAt > RETRANSMIT_TIMEOUT_MS)) {
      Serial.printf("[BLE] 🔄 Retransmitting seq %u\n", (uint8_t)(seqNo - 1));
      pStreamChar->notify(lastPacketBuf, lastPacketLen);
      lastSentAt = now;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------------- Public API ----------------
void ble_setup() {
  if (!snapshotMutex) snapshotMutex = xSemaphoreCreateMutex();

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║    🚀 INITIALIZING NimBLE SERVER      ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.printf("[BLE] Device Name: %s\n", DEVICE_NAME);
  
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  
  // CRITICAL: Set max MTU for better throughput
  NimBLEDevice::setMTU(512);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  auto addr = NimBLEDevice::getAddress();
  Serial.printf("[BLE] 📍 BLE Address: %s\n", addr.toString().c_str());

  NimBLEService* svc = pServer->createService(TELEMETRY_SVC_UUID);

  // Stream characteristic with callbacks
  pStreamChar = svc->createCharacteristic(
    TELEMETRY_STREAM_UUID,
    NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
  );
  pStreamChar->setCallbacks(new StreamCharCallbacks());

  // Control characteristic
  pControlChar = svc->createCharacteristic(
    CONTROL_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pControlChar->setCallbacks(new ControlCallback());

  svc->start();

  pAdvertising = NimBLEDevice::getAdvertising();

  Serial.println("\n[BLE] 📋 Service Configuration:");
  Serial.printf("  Service UUID:  %s\n", TELEMETRY_SVC_UUID);
  Serial.printf("  Stream UUID:   %s\n", TELEMETRY_STREAM_UUID);
  Serial.printf("  Control UUID:  %s\n", CONTROL_UUID);
  Serial.println("\n════════════════════════════════════════\n");

  startupCaptureUntil = now_ms() + STARTUP_SNAPSHOT_DELAY_MS;
  startupSnapshotCaptured = false;
  lastManufacturerUpdateAt = 0;
  buildAndSetAdvertFromSnapshot();
}

void ble_start() {
  startAdvertising();
  xTaskCreatePinnedToCore(bleTask, "BLETask", 8192, NULL, 5, NULL, 0);
  Serial.println("[BLE] ✅ BLE Service fully started and advertising");
}

void ble_publish_snapshot(const Telemetry* snapshot) {
  if (!snapshot) return;
  if (!snapshotMutex) snapshotMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(snapshotMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    lastSnapshot = *snapshot;
    xSemaphoreGive(snapshotMutex);
  }
  // Only update advertising if not connected (avoid disrupting connection)
  if (!deviceConnected) {
    buildAndSetAdvertFromSnapshot();
  }
}

static void handleAck(uint8_t ackSeq) {
  if (awaitingAck && ackSeq == (uint8_t)(seqNo - 1)) {
    awaitingAck = false;
    Serial.printf("[BLE] ✅ ACK received for seq %u\n", ackSeq);
  }
}