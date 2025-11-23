// ble_telemetry.cpp
// EXTENDED: Added mesh network relay characteristic support
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
// 🔹 NEW: Mesh relay characteristic UUID
static const char* MESH_RELAY_UUID       = "0000A104-0000-1000-8000-00805F9B34FB";

static const uint32_t CONNECTABLE_WINDOW_MS = 30 * 1000UL;
static const uint32_t ADVERT_INTERVAL_MS_NORMAL = 500;
static const uint32_t ADVERT_INTERVAL_MS_URGENT = 120;

// CRITICAL: Keep connectable for testing
static const bool FORCE_CONNECTABLE_FOR_TESTING = true;
static const bool REQUIRE_APP_AUTH = false;
static const uint32_t APP_AUTH_TOKEN = 0xA1B2C3D4;

// ---------------- Alert Thresholds ----------------
// Vitals thresholds
static const float BPM_LOW_THRESHOLD = 60.0f;
static const float BPM_HIGH_THRESHOLD = 100.0f;
static const float SPO2_LOW_THRESHOLD = 90.0f;
static const float TEMP_HIGH_THRESHOLD = 38.0f;  // 38°C fever threshold

// CO2 thresholds
// ------------------ MQ Configuration -----------------
constexpr float ROOM_BASELINE_PPM = 385.0f;  // measured baseline in your room
static constexpr float CO2_HIGH_THRESHOLD = ROOM_BASELINE_PPM + (1500.0f - 400.0f);   // General high CO2
static constexpr float CO2_SMOKE_THRESHOLD = ROOM_BASELINE_PPM + (2500.0f - 400.0f);   // Smoke/fire detection threshold

// Alert smoothing
static const int CONSECUTIVE_FOR_ABNORMAL = 3;
static const int CONSECUTIVE_FOR_GPS_LOSS = 3;
static const int CONSECUTIVE_FOR_SMOKE = 2;
static const int HOLD_MS_IMU_EVENTS = 3000;

// ---------------- internal state ----------------
static Telemetry lastSnapshot;
static SemaphoreHandle_t snapshotMutex = nullptr;

static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pStreamChar = nullptr;
static NimBLECharacteristic* pControlChar = nullptr;
// 🔹 NEW: Mesh relay characteristic
static NimBLECharacteristic* pMeshRelayChar = nullptr;
static NimBLEAdvertising* pAdvertising = nullptr;

static bool deviceConnected = false;
static bool clientAuthorized = false;
static bool clientSubscribed = false;
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

// Alert counters
static int lowBpmCount = 0;
static int highBpmCount = 0;
static int lowSpO2Count = 0;
static int co2HighCount = 0;
static int co2SmokeCount = 0;
static int highTempCount = 0;
static int gpsGoodCount = 0;
static int gpsBadCount = 0;

// IMU event timestamps
static uint32_t lastFreefallSeenAt = 0;
static uint32_t lastVibrationSeenAt = 0;
static uint32_t lastShockSeenAt = 0;
static uint32_t lastFallSeenAt = 0;
static uint32_t lastTiltSeenAt = 0;

// 🔹 NEW: Mesh relay tracking
static uint32_t meshPacketsReceived = 0;
static uint32_t meshPacketsRelayed = 0;

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

// 🔹 NEW: Mesh relay callback handler
class MeshRelayCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr) {
    std::string data = chr->getValue();
    
    if (data.empty()) {
      Serial.println("[MESH] ⚠️ Received empty mesh packet");
      return;
    }
    
    meshPacketsReceived++;
    
    Serial.printf("[MESH] 📦 Received mesh packet: %d bytes (Total: %u)\n", 
                  data.size(), meshPacketsReceived);
    
    // Basic packet validation
    if (data.size() < 10) {
      Serial.println("[MESH] ⚠️ Packet too small, ignoring");
      return;
    }
    
    // Parse basic mesh packet info (first few bytes should contain header)
    uint8_t hopCount = (data.size() > 8) ? (uint8_t)data[8] : 0;
    
    Serial.printf("[MESH] 📡 Hop count: %d\n", hopCount);
    
    // Check if we should relay (max hop limit)
    const uint8_t MAX_HOPS = 10;
    if (hopCount >= MAX_HOPS) {
      Serial.println("[MESH] 🚫 Max hops reached, not relaying");
      return;
    }
    
    // For now, just log receipt. In full implementation:
    // 1. Parse packet completely
    // 2. Check if already seen (prevent loops)
    // 3. Add our device to hop list
    // 4. Broadcast to nearby devices
    
    meshPacketsRelayed++;
    Serial.printf("[MESH] ✅ Packet processed (Relayed: %u)\n", meshPacketsRelayed);
    
    // Notify other connected devices via characteristic
    if (pMeshRelayChar && deviceConnected) {
      pMeshRelayChar->notify((uint8_t*)data.c_str(), data.size());
      Serial.println("[MESH] 📢 Forwarded to connected clients");
    }
  }
  
  void onRead(NimBLECharacteristic* chr) {
    Serial.println("[MESH] 📖 Client READ mesh relay characteristic");
  }
};

// ---------------- NimBLE Server callbacks ----------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, ble_gap_conn_desc* desc) {
    deviceConnected = true;
    clientAuthorized = true;
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║     ✅ CLIENT CONNECTED TO ESP32      ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.printf("[BLE] Device Name: %s\n", DEVICE_NAME);
    Serial.printf("[BLE] Client Address: %s\n", NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    Serial.printf("[BLE] Connection ID: %d\n", desc->conn_handle);
    Serial.printf("[BLE] Status: connected=%d auth=%d\n", deviceConnected, clientAuthorized);
    Serial.println("════════════════════════════════════════\n");
    
    server->updateConnParams(desc->conn_handle, 24, 48, 0, 400);
  }
  
  void onDisconnect(NimBLEServer* server) {
    deviceConnected = false;
    clientAuthorized = false;
    clientSubscribed = false;
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║        ❌ CLIENT DISCONNECTED          ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    delay(500);
    if (pAdvertising) {
      startAdvertising();
    }
  }
};

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

class ControlCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr) {
    std::string v = chr->getValue();
    
    Serial.printf("[BLE] 📝 Control write received: %d bytes\n", v.size());

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

// ---------------- IMPROVED ALERT EVALUATION ----------------
static void evaluateAlertsFromSnapshot(Telemetry &s) {
  uint32_t now = now_ms();
  
  // ===== IMU EVENT HOLD LOGIC =====
  // Update last seen timestamps for IMU events
  if (s.freefall) lastFreefallSeenAt = now;
  if (s.vibration) lastVibrationSeenAt = now;
  if (s.shock) lastShockSeenAt = now;
  if (s.fall) lastFallSeenAt = now;
  if (s.tilt) lastTiltSeenAt = now;

  // Check if events are still within hold period
  bool heldFreefall = (lastFreefallSeenAt != 0) && ((int32_t)(now - lastFreefallSeenAt) < HOLD_MS_IMU_EVENTS);
  bool heldVibration = (lastVibrationSeenAt != 0) && ((int32_t)(now - lastVibrationSeenAt) < HOLD_MS_IMU_EVENTS);
  bool heldShock = (lastShockSeenAt != 0) && ((int32_t)(now - lastShockSeenAt) < HOLD_MS_IMU_EVENTS);
  bool heldFall = (lastFallSeenAt != 0) && ((int32_t)(now - lastFallSeenAt) < HOLD_MS_IMU_EVENTS);
  bool heldTilt = (lastTiltSeenAt != 0) && ((int32_t)(now - lastTiltSeenAt) < HOLD_MS_IMU_EVENTS);

  // Apply held states back to snapshot
  s.freefall = heldFreefall;
  s.vibration = heldVibration;
  s.shock = heldShock;
  s.fall = heldFall;
  s.tilt = heldTilt;

  // ===== VITALS VALIDATION =====
  float bpm = s.bpm;
  float spo2 = s.spo2;
  float temp = s.tempBME;
  float co2 = s.co2ppm;
  
  bool bpmValid = (bpm > 0.5f && isfinite(bpm));
  bool spo2Valid = (spo2 > 0.5f && isfinite(spo2));
  bool tempValid = isfinite(temp);
  bool co2Valid = isfinite(co2);

  // ===== BPM ALERT LOGIC =====
  if (bpmValid && (bpm < BPM_LOW_THRESHOLD)) {
    lowBpmCount++;
  } else {
    lowBpmCount = 0;
  }
  
  if (bpmValid && (bpm > BPM_HIGH_THRESHOLD)) {
    highBpmCount++;
  } else {
    highBpmCount = 0;
  }

  // ===== SPO2 ALERT LOGIC =====
  if (spo2Valid && (spo2 < SPO2_LOW_THRESHOLD)) {
    lowSpO2Count++;
  } else {
    lowSpO2Count = 0;
  }

  // ===== TEMPERATURE ALERT LOGIC =====
  if (tempValid && (temp > TEMP_HIGH_THRESHOLD)) {
    highTempCount++;
  } else {
    highTempCount = 0;
  }

  // ===== CO2 ALERT LOGIC =====
  if (co2Valid && (co2 > CO2_HIGH_THRESHOLD)) {
    co2HighCount++;
  } else {
    co2HighCount = 0;
  }

  // ===== SMOKE/FIRE DETECTION =====
  if (co2Valid && (co2 > CO2_SMOKE_THRESHOLD)) {
    co2SmokeCount++;
  } else {
    co2SmokeCount = 0;
  }
  
  // Set smoke/fire flag if threshold reached
  s.smokeOrFire = (co2SmokeCount >= CONSECUTIVE_FOR_SMOKE);

  // ===== ABNORMAL VITALS DETECTION =====
  bool abnormalBpm = (lowBpmCount >= CONSECUTIVE_FOR_ABNORMAL) || 
                     (highBpmCount >= CONSECUTIVE_FOR_ABNORMAL);
  bool abnormalSpO2 = (lowSpO2Count >= CONSECUTIVE_FOR_ABNORMAL);
  bool abnormalCO2 = (co2HighCount >= CONSECUTIVE_FOR_ABNORMAL);
  bool abnormalTemp = (highTempCount >= CONSECUTIVE_FOR_ABNORMAL);
  
  bool abnormalVitals = abnormalBpm || abnormalSpO2 || abnormalCO2 || abnormalTemp;

  // ===== GPS VALIDATION =====
  bool gpsRawOK = s.gpsOK && 
                  isfinite(s.lat) && 
                  isfinite(s.lon) && 
                  (fabs(s.lat) > 1e-7 || fabs(s.lon) > 1e-7);
  
  if (gpsRawOK) {
    gpsGoodCount++;
    gpsBadCount = 0;
  } else {
    gpsBadCount++;
    gpsGoodCount = 0;
  }
  
  bool gpsValid = (gpsGoodCount >= 1);
  if (gpsBadCount >= CONSECUTIVE_FOR_GPS_LOSS) {
    gpsValid = false;
  }
  
  s.gpsOK = gpsValid;

  // ===== URGENT FLAG LOGIC =====
  // Urgent if any critical condition detected
  bool criticalFall = s.fall || s.freefall;
  bool criticalVitals = abnormalVitals;
  bool criticalSmoke = s.smokeOrFire;
  bool criticalVibration = s.vibration;
  
  s.urgent = criticalFall || criticalVitals || criticalSmoke || criticalVibration;

  // ===== DEBUG LOGGING (when alerts trigger) =====
  if (s.urgent) {
    Serial.println("\n⚠️⚠️⚠️ URGENT ALERT TRIGGERED ⚠️⚠️⚠️");
    if (criticalFall) {
      Serial.println("  🚨 FALL DETECTED");
    }
    if (abnormalBpm) {
      Serial.printf("  💓 ABNORMAL BPM: %.1f (Low: %d, High: %d)\n", 
                    bpm, lowBpmCount, highBpmCount);
    }
    if (abnormalSpO2) {
      Serial.printf("  🫁 LOW SPO2: %.1f%% (Count: %d)\n", spo2, lowSpO2Count);
    }
    if (abnormalCO2) {
      Serial.printf("  💨 HIGH CO2: %.1f ppm (Count: %d)\n", co2, co2HighCount);
    }
    if (abnormalTemp) {
      Serial.printf("  🌡️ HIGH TEMP: %.1f°C (Count: %d)\n", temp, highTempCount);
    }
    if (criticalSmoke) {
      Serial.printf("  🔥 SMOKE/FIRE DETECTED: %.1f ppm (Count: %d)\n", 
                    co2, co2SmokeCount);
    }
    if (s.vibration) {
      Serial.println("  📳 VIBRATION DETECTED");
    }
    Serial.println("════════════════════════════════════════\n");
  }
}

static void requestConnectableWindow(uint32_t ms) {
  uint32_t t = now_ms();
  if (connectableUntil == 0 || t + ms > connectableUntil) connectableUntil = t + ms;
}

static void startAdvertising() {
  if (!pAdvertising) return;
  
  pAdvertising->stop();
  delay(100);
  
  if (FORCE_CONNECTABLE_FOR_TESTING) {
    Serial.println("[BLE] 📡 Starting CONNECTABLE advertising (TESTING MODE)");
    isCurrentlyConnectable = true;
  } else {
    Serial.println("[BLE] 📡 Starting advertising");
  }
  
  NimBLEAdvertisementData advData;
  advData.setName(DEVICE_NAME);
  advData.addServiceUUID(TELEMETRY_SVC_UUID);
  pAdvertising->setAdvertisementData(advData);
  
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
  
  // Build flags byte with correct bit positions
  uint8_t flags = 0;
  if (s.freefall) flags |= (1<<0);      // Bit 0
  if (s.vibration) flags |= (1<<1);     // Bit 1
  if (s.tilt) flags |= (1<<2);          // Bit 2
  if (s.fall) flags |= (1<<3);          // Bit 3
  if (s.shock) flags |= (1<<4);         // Bit 4
  if (s.urgent) flags |= (1<<5);        // Bit 5 - URGENT
  if (s.gpsOK) flags |= (1<<6);         // Bit 6 - GPS
  if (s.smokeOrFire) flags |= (1<<7);   // Bit 7 - SMOKE/FIRE
  
  buf[3] = flags;

  buf[4] = 100; // Battery placeholder
  buf[5] = (uint8_t)((s.bpm > 0.5f && isfinite(s.bpm)) ? constrain((int)round(s.bpm), 0, 255) : 0);
  buf[6] = (uint8_t)((s.spo2 > 0.5f && isfinite(s.spo2)) ? constrain((int)round(s.spo2), 0, 255) : 0);
  uint16_t co2 = (uint16_t)constrain((int)round(isnan(s.co2ppm) ? 0 : s.co2ppm), 0, 65535);
  buf[7] = co2 & 0xFF; 
  buf[8] = (co2>>8) & 0xFF;
  int16_t temp_centi = (int16_t)round(isnan(s.tempBME) ? 0 : (s.tempBME * 100.0f));
  buf[9] = temp_centi & 0xFF; 
  buf[10] = (temp_centi >> 8) & 0xFF;

  int32_t lat_e5 = (int32_t)round(s.lat * 1e5);
  int32_t lon_e5 = (int32_t)round(s.lon * 1e5);
  writeInt24LE(&buf[11], lat_e5);
  writeInt24LE(&buf[14], lon_e5);

  uint32_t ts = (uint32_t)((millis()/1000) & 0xFFFFFF);
  buf[17] = ts & 0xFF; 
  buf[18] = (ts>>8) & 0xFF; 
  buf[19] = (ts>>16) & 0xFF;

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
    if (s.urgent) requestConnectableWindow(CONNECTABLE_WINDOW_MS);
    return;
  }

  if ((now - lastManufacturerUpdateAt) < MANUF_UPDATE_INTERVAL_MS) return;

  lastManufacturerData = man;
  lastManufacturerUpdateAt = now;
  
  if (!deviceConnected && pAdvertising) {
    startAdvertising();
  }
}

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
  packet[idx++] = 0xA1; 
  packet[idx++] = 0xB2;
  packet[idx++] = 1; // type
  packet[idx++] = seqNo++;
  uint64_t ts = (uint64_t)millis();
  for (int i = 0; i < 6; ++i) packet[idx++] = (ts >> (8*i)) & 0xFF;
  packet[idx++] = (uint8_t)((s.bpm > 0.5f && isfinite(s.bpm)) ? constrain((int)round(s.bpm), 0, 255) : 0);
  packet[idx++] = (uint8_t)((s.spo2 > 0.5f && isfinite(s.spo2)) ? constrain((int)round(s.spo2), 0, 255) : 0);
  uint16_t co2v = (uint16_t)constrain((int)round(isnan(s.co2ppm)?0:s.co2ppm), 0, 65535);
  packet[idx++] = co2v & 0xFF; 
  packet[idx++] = (co2v>>8) & 0xFF;
  
  // Build flags with smoke/fire
  uint8_t flags = 0;
  if (s.freefall) flags |= (1<<0);
  if (s.vibration) flags |= (1<<1);
  if (s.tilt) flags |= (1<<2);
  if (s.fall) flags |= (1<<3);
  if (s.shock) flags |= (1<<4);
  if (s.urgent) flags |= (1<<5);
  if (s.smokeOrFire) flags |= (1<<6);
  
  packet[idx++] = flags;

  memcpy(lastPacketBuf, packet, idx);
  lastPacketLen = idx;

  bool sent = pStreamChar->notify(lastPacketBuf, lastPacketLen);
  
  if (sent) {
    Serial.printf("[BLE] 📡 Notification #%d sent: BPM=%d SpO2=%d CO2=%d%s%s\n", 
                  (int)(seqNo-1), (int)s.bpm, (int)s.spo2, (int)s.co2ppm,
                  s.urgent ? " [URGENT]" : "",
                  s.smokeOrFire ? " [SMOKE/FIRE]" : "");
  } else {
    Serial.println("[BLE] ❌ Notification FAILED to send");
  }
  
  lastSentAt = now_ms();
  awaitingAck = true;
}

// ---------------- BLE background task ----------------
static void bleTask(void* pv) {
  const TickType_t advertPeriod = pdMS_TO_TICKS(5000);
  const TickType_t notifyPeriod = pdMS_TO_TICKS(1000);
  TickType_t lastAdv = xTaskGetTickCount();
  TickType_t lastNotify = xTaskGetTickCount();

  Serial.println("[BLE] 🚀 BLE Task started");

  while (true) {
    TickType_t nowT = xTaskGetTickCount();
    uint32_t now = now_ms();

    if (!deviceConnected && (nowT - lastAdv >= advertPeriod)) {
      lastAdv = nowT;
      buildAndSetAdvertFromSnapshot();
    }

    if (deviceConnected && clientSubscribed && (nowT - lastNotify >= notifyPeriod)) {
      lastNotify = nowT;
      sendNotificationIfConnected();
    }

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
  NimBLEDevice::setMTU(512);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  auto addr = NimBLEDevice::getAddress();
  Serial.printf("[BLE] 📍 BLE Address: %s\n", addr.toString().c_str());

  NimBLEService* svc = pServer->createService(TELEMETRY_SVC_UUID);

  pStreamChar = svc->createCharacteristic(
    TELEMETRY_STREAM_UUID,
    NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
  );
  pStreamChar->setCallbacks(new StreamCharCallbacks());

  pControlChar = svc->createCharacteristic(
    CONTROL_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pControlChar->setCallbacks(new ControlCallback());

  // 🔹 NEW: Create mesh relay characteristic
  pMeshRelayChar = svc->createCharacteristic(
    MESH_RELAY_UUID,
    NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY
  );
  pMeshRelayChar->setCallbacks(new MeshRelayCallback());
  Serial.println("[MESH] ✅ Mesh relay characteristic created");

  svc->start();

  pAdvertising = NimBLEDevice::getAdvertising();

  Serial.println("\n[BLE] 📋 Service Configuration:");
  Serial.printf("  Service UUID:  %s\n", TELEMETRY_SVC_UUID);
  Serial.printf("  Stream UUID:   %s\n", TELEMETRY_STREAM_UUID);
  Serial.printf("  Control UUID:  %s\n", CONTROL_UUID);
  Serial.printf("  Mesh Relay:    %s\n", MESH_RELAY_UUID);
  Serial.println("\n[BLE] 🚨 Alert Thresholds:");
  Serial.printf("  BPM Low:       %.1f\n", BPM_LOW_THRESHOLD);
  Serial.printf("  BPM High:      %.1f\n", BPM_HIGH_THRESHOLD);
  Serial.printf("  SpO2 Low:      %.1f%%\n", SPO2_LOW_THRESHOLD);
  Serial.printf("  Temp High:     %.1f°C\n", TEMP_HIGH_THRESHOLD);
  Serial.printf("  CO2 High:      %.1f ppm\n", CO2_HIGH_THRESHOLD);
  Serial.printf("  CO2 Smoke:     %.1f ppm (FIRE/SMOKE)\n", CO2_SMOKE_THRESHOLD);
  Serial.printf("  Consecutive:   %d readings\n", CONSECUTIVE_FOR_ABNORMAL);
  Serial.println("\n[MESH] 📡 Mesh Network:");
  Serial.println("  Status:        ENABLED");
  Serial.println("  Max Hops:      10");
  Serial.println("  Mode:          Relay + Forward");
  Serial.println("════════════════════════════════════════\n");

  startupCaptureUntil = now_ms() + STARTUP_SNAPSHOT_DELAY_MS;
  startupSnapshotCaptured = false;
  lastManufacturerUpdateAt = 0;
  buildAndSetAdvertFromSnapshot();
}

void ble_start() {
  startAdvertising();
  xTaskCreatePinnedToCore(bleTask, "BLETask", 8192, NULL, 5, NULL, 0);
  Serial.println("[BLE] ✅ BLE Service fully started and advertising");
  Serial.println("[MESH] 🌐 Mesh relay ready for incoming packets");
}

void ble_publish_snapshot(const Telemetry* snapshot) {
  if (!snapshot) return;
  if (!snapshotMutex) snapshotMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(snapshotMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    lastSnapshot = *snapshot;
    xSemaphoreGive(snapshotMutex);
  }
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

// 🔹 NEW: Get mesh relay statistics
void ble_get_mesh_stats(uint32_t* received, uint32_t* relayed) {
  if (received) *received = meshPacketsReceived;
  if (relayed) *relayed = meshPacketsRelayed;
}