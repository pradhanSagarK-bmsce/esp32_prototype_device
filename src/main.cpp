// Multitask Sensor Hub (Modern Clean UI) - Improved layout + Alert animations
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "IMU.h"
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include "MAX30105.h"
#include "PulseAlgorithm.h"
#include "telemetry_shared.h"
#include "ble_telemetry.h"
#include <U8g2lib.h>
#include <math.h>

// ----------------- Pins & I2C -----------------
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t BME_SDA  = 32;
constexpr uint8_t BME_SCL  = 33;
constexpr uint8_t GPS_RX_PIN = 16;
constexpr uint8_t GPS_TX_PIN = 17;
constexpr uint32_t GPS_BAUD = 9600;
constexpr uint8_t MQ135_PIN = 34;
constexpr uint8_t BME_ADDR = 0x76;

// UI Constants (tunable)
constexpr int SCR_W = 128;
constexpr int SCR_H = 64;
constexpr int MARGIN = 4;
constexpr int HEADER_H = 12;
constexpr int STATUS_ICON_SIZE = 4;
constexpr int GAP = 4;
constexpr int SMALL_FONT_H = 6;
constexpr int MED_FONT_H = 10;
constexpr int LARGE_FONT_H = 14;

// ----------------- Devices -----------------
Adafruit_BME280 bme;
TwoWire I2CBME = TwoWire(1);
IMU imu;
HardwareSerial GPSs(2);
TinyGPSPlus gps;
MAX30105 particleSensor;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ----------------- RTOS resources -----------------
SemaphoreHandle_t i2cMutex = nullptr;
SemaphoreHandle_t dataMutex = nullptr;

// ----------------- Telemetry (shared) -----------------
Telemetry telem;

// Display state
static uint8_t oledPage = 0;
constexpr int DISP_WIDTH = 128;

// ----------------- Task prototypes -----------------
void maxTask(void* pv);
void imuTask(void* pv);
void gpsTask(void* pv);
void bmeTask(void* pv);
void mqTask(void* pv);
void calcTask(void* pv);
void oledTask(void* pv);

// ----------------- Helpers -----------------
void safeI2cBegin() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
}

bool safeParticleBegin() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    bool ok = particleSensor.begin(Wire, I2C_SPEED_STANDARD);
    xSemaphoreGive(i2cMutex);
    return ok;
  } else {
    return particleSensor.begin(Wire, I2C_SPEED_STANDARD);
  }
}

void safeParticleSetup() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    particleSensor.setup(50, 4, 2, 100, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x24);
    particleSensor.setPulseAmplitudeIR(0x24);
    xSemaphoreGive(i2cMutex);
  } else {
    particleSensor.setup(50, 4, 2, 100, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x24);
    particleSensor.setPulseAmplitudeIR(0x24);
  }
}

// SMALL utility to safely format into buffer
static void safeSprintf(char *dst, size_t size, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(dst, size, fmt, ap);
  va_end(ap);
}

// ---------- Display helpers with safe layout ----------
void drawHeader(const char* title) {
  u8g2.setFont(u8g2_font_helvB08_tr); // bold small
  int w = u8g2.getStrWidth(title);
  int x = MARGIN;
  int y = HEADER_H - 2;
  u8g2.drawStr(x, y, title);
  // header underline
  u8g2.drawHLine(MARGIN, HEADER_H, SCR_W - 2*MARGIN);
}

void drawStatusBar(bool leftToRight = false) {
  // small indicators on top-right, spaced
  int x = SCR_W - MARGIN - STATUS_ICON_SIZE;
  const int y = 6;
  // order: MAX, IMU, BME, GPS (from right->left)
  // we keep tight spacing and draw small dots or circles
  auto drawIcon = [&](bool ok) {
    if (ok) {
      u8g2.drawDisc(x, y, STATUS_ICON_SIZE/2);
    } else {
      u8g2.drawCircle(x, y, STATUS_ICON_SIZE/2);
    }
    x -= (STATUS_ICON_SIZE + 6);
  };
  drawIcon(telem.maxOK);
  drawIcon(telem.imuOK);
  drawIcon(telem.bmeOK);
  drawIcon(telem.gpsOK);
}

// Label-left, value-right inside a given column rectangle (x,w)
void drawLabelValueBox(int x, int y, int w, const char* label, const char* value, const char* unit = "") {
  // label small font on left, value bold on right (within column)
  u8g2.setFont(u8g2_font_tom_thumb_4x6_tr);
  u8g2.drawStr(x + 2, y, label);
  u8g2.setFont(u8g2_font_helvB08_tr);
  char combined[32];
  safeSprintf(combined, sizeof(combined), "%s%s", value, unit);
  int valW = u8g2.getStrWidth(combined);
  int vx = x + w - 2 - valW;
  if (vx < x + 30) vx = x + 30; // prevent overrun into label
  u8g2.drawStr(vx, y, combined);
}

// metric box with centered label on top and value at bottom center
void drawMetricBoxSafe(int x, int y, int w, int h, const char* label, const char* value, const char* unit = "") {
  u8g2.drawRFrame(x, y, w, h, 2);
  u8g2.setFont(u8g2_font_tom_thumb_4x6_tr);
  int labelW = u8g2.getStrWidth(label);
  u8g2.drawStr(x + (w - labelW) / 2, y + 7, label);
  u8g2.setFont(u8g2_font_helvB10_tr);
  char combined[32];
  safeSprintf(combined, sizeof(combined), "%s%s", value, unit);
  int valW = u8g2.getStrWidth(combined);
  u8g2.drawStr(x + (w - valW) / 2, y + h - 4, combined);
}

// Choose alert by priority: vib > fall > shock > freefall > tilt
struct Alert {
  bool active;
  uint8_t priority; // 1 highest
  const char* text;
};
static Alert pickAlert(const Telemetry& s) {
  if (s.vibration) return {true, 1, "! VIBRATION" };
  if (s.fall)      return {true, 2, "! FALL DETECTED" };
  if (s.shock)     return {true, 3, "! SHOCK" };
  if (s.freefall)  return {true, 4, "! FREEFALL" };
  if (s.tilt)      return {true, 5, "! TILT" };
  return {false, 255, ""};
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println("\n=== Multitask Sensor Hub (Modern UI) ===");

  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  if (!i2cMutex || !dataMutex) {
    Serial.println("Mutex creation failed!");
    while(1) delay(1000);
  }

  safeI2cBegin();
  I2CBME.begin(BME_SDA, BME_SCL, 400000);

  if (!safeParticleBegin()) {
    Serial.println("MAX30105 init failed; will retry in task.");
    telem.maxOK = false;
  } else {
    safeParticleSetup();
    resetPulseBuffers();
    telem.maxOK = true;
    Serial.println("MAX30105 init OK");
  }

  bool okBME = bme.begin(BME_ADDR, &I2CBME);
  telem.bmeOK = okBME;
  Serial.println(okBME ? "BME280 OK" : "BME280 not found");

  imu.begin();
  imu.calibrateGyro();
  telem.imuOK = true;
  Serial.println("IMU init done");

  GPSs.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS UART started");

  // OLED boot with modern splash
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.drawStr(24, 24, "SENSOR HUB");
  u8g2.setFont(u8g2_font_tom_thumb_4x6_tr);
  u8g2.drawStr(38, 36, "Initializing...");
  u8g2.drawRFrame(8, 8, 112, 48, 3);
  u8g2.sendBuffer();
  delay(1100);

  ble_setup();
  ble_start(); // creates BLETask
  // Create tasks
  xTaskCreatePinnedToCore(maxTask, "MAXTask", 8192, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(imuTask, "IMUTask", 6144, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(calcTask, "CalcTask", 6144, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(gpsTask, "GPSTask", 6144, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(bmeTask, "BMETask", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(mqTask, "MQTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(oledTask, "OLEDTask", 8192, NULL, 3, NULL, 1);

  Serial.println("Tasks created. Entering idle.");
}

// ----------------- MAX Task -----------------
void maxTask(void* pv) {
  const TickType_t samplePeriod = pdMS_TO_TICKS(10);
  TickType_t lastWake = xTaskGetTickCount();
  TickType_t lastReinitTry = xTaskGetTickCount();
  const TickType_t reinitPeriod = pdMS_TO_TICKS(5000);

  for (;;) {
    vTaskDelayUntil(&lastWake, samplePeriod);
    if (!telem.maxOK && (xTaskGetTickCount()-lastReinitTry >= reinitPeriod)) {
      lastReinitTry = xTaskGetTickCount();
      if (safeParticleBegin()) {
        safeParticleSetup();
        resetPulseBuffers();
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))==pdTRUE) {
          telem.maxOK = true; xSemaphoreGive(dataMutex);
        } else telem.maxOK = true;
        Serial.println("MAX30105 re-init OK");
      }
    }

    if (!telem.maxOK) continue;

    int32_t ir=0, red=0;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5))==pdTRUE) {
      ir = particleSensor.getIR();
      red = particleSensor.getRed();
      xSemaphoreGive(i2cMutex);
    } else continue;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))==pdTRUE) {
      irBuffer[bufferIdx]  = ir;
      redBuffer[bufferIdx] = red;
      bufferIdx = (bufferIdx + 1) % BUFFER_SIZE;
      if (bufferIdx == 0) bufferFull = true;
      telem.lastIR = ir;
      telem.lastRED = red;
      telem.maxOK = true;
      xSemaphoreGive(dataMutex);
    }
  }
}

// ----------------- IMU Task -----------------
void imuTask(void* pv) {
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    bool ok = false;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))==pdTRUE) { ok = imu.update(); xSemaphoreGive(i2cMutex); }
    else ok = imu.update();

    if (ok) {
      ImuEuler e = imu.getEuler();
      ImuRaw r = imu.getRaw();
      ImuEvents ev = imu.getEvents();
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))==pdTRUE) {
        telem.roll = e.roll; telem.pitch = e.pitch; telem.yaw = e.yaw;
        telem.ax = r.ax_g; telem.ay = r.ay_g; telem.az = r.az_g;
        telem.gx = r.gx_dps; telem.gy = r.gy_dps; telem.gz = r.gz_dps;
        telem.freefall = ev.freefall;
        telem.shock = ev.shock;
        telem.tilt = ev.tilt;
        telem.vibration = ev.vibration;
        telem.fall = ev.fall;
        telem.imuOK = true;
        xSemaphoreGive(dataMutex);
      }
    } else { if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5))==pdTRUE) { telem.imuOK=false; xSemaphoreGive(dataMutex); } }
  }
}

// ----------------- Calc Task -----------------
void calcTask(void* pv) {
  const TickType_t period = pdMS_TO_TICKS(1000);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    float newBPM = calculateBPM();
    float newSpO2 = calculateSpO2();
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))==pdTRUE) { telem.bpm=newBPM; telem.spo2=newSpO2; xSemaphoreGive(dataMutex); }
    Serial.printf("Calc: BPM=%.1f SpO2=%.1f\n", newBPM,newSpO2);
  }
}

// ----------------- GPS Task -----------------
void gpsTask(void* pv) {
  const TickType_t propagatePeriod = pdMS_TO_TICKS(10000);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    while(GPSs.available()) gps.encode((char)GPSs.read());
    if ((xTaskGetTickCount()-lastWake) >= propagatePeriod) {
      lastWake = xTaskGetTickCount();
      if (gps.location.isValid() && xSemaphoreTake(dataMutex,pdMS_TO_TICKS(50))==pdTRUE) {
        telem.lat = gps.location.lat(); telem.lon = gps.location.lng();
        telem.alt = gps.altitude.meters(); telem.speed = gps.speed.kmph();
        telem.gpsOK=true; xSemaphoreGive(dataMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ----------------- BME Task -----------------
void bmeTask(void* pv) {
  const TickType_t period = pdMS_TO_TICKS(2500);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    float t=bme.readTemperature(), h=bme.readHumidity(), p=bme.readPressure()/100.0F;
    if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(20))==pdTRUE) { telem.tempBME=t; telem.humBME=h; telem.pressBME=p; telem.bmeOK=true; xSemaphoreGive(dataMutex); }
  }
}

// ----------------- MQ Task -----------------
void mqTask(void* pv) {
  const TickType_t period = pdMS_TO_TICKS(3500);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    int raw = analogRead(MQ135_PIN);
    float voltage = raw*(3.3f/4095.0f);
    float co2 = 116.6020682f*pow((voltage/3.3f), -2.769034857f);
    if (xSemaphoreTake(dataMutex,pdMS_TO_TICKS(20))==pdTRUE) { telem.co2ppm=co2; xSemaphoreGive(dataMutex); }
  }
}

// ----------------- OLED Task -----------------
void oledTask(void* pv) {
    const TickType_t framePeriod = pdMS_TO_TICKS(50);
    TickType_t lastWake = xTaskGetTickCount();
    static int32_t dispBuf[DISP_WIDTH];
    static int idx = 0;
    for (int i = 0; i < DISP_WIDTH; i++) dispBuf[i] = 0;

    const uint8_t totalPages = 5; // now we have 5 pages
    uint32_t framesPerPage = 5000 / 50; // 5 seconds per page
    uint32_t frameCounter = 0;

    for (;;) {
        vTaskDelayUntil(&lastWake, framePeriod);
        frameCounter++;

        static uint32_t lastDebugTime = 0;
        uint32_t now = millis();
        
        Telemetry snap;
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            snap = telem;
            xSemaphoreGive(dataMutex);
            
            // Debug print every 5 seconds
            if (now - lastDebugTime >= 5000) {
                Serial.printf("[OLED] Publishing: BPM=%.1f SPO2=%.1f CO2=%.1f\n",
                            snap.bpm, snap.spo2, snap.co2ppm);
                lastDebugTime = now;
            }
            
            // push snapshot to BLE module (non-blocking)
            ble_publish_snapshot(&snap);
        } else {
            Serial.println("[OLED] Failed to take dataMutex!");
        }

        // Push latest IR into rolling buffer (safe)
        dispBuf[idx] = snap.lastIR > 0 ? snap.lastIR : 0;
        idx = (idx + 1) % DISP_WIDTH;

        u8g2.clearBuffer();

        // Page cycling
        if ((frameCounter % framesPerPage) == 0) oledPage = (oledPage + 1) % totalPages;

        // Draw header and status bar
        const char* pageTitles[totalPages] = {"VITALS", "MOTION", "ENVIRONMENT", "AIR QUALITY", "LOCATION"};
        drawHeader(pageTitles[oledPage]);
        drawStatusBar();

        // Alert banner
        Alert a = pickAlert(snap);
        bool showAlert = a.active;
        int contentY = HEADER_H + 2;
        if (showAlert) {
            contentY += 12;
            bool phase = ((frameCounter / 6) % 2) == 0;
            int bannerY = HEADER_H + 2;
            int bannerH = 10;
            if (phase) {
                u8g2.drawRBox(MARGIN, bannerY, SCR_W - 2 * MARGIN, bannerH, 2);
                u8g2.setDrawColor(0);
                u8g2.setFont(u8g2_font_helvB08_tr);
                int tw = u8g2.getStrWidth(a.text);
                u8g2.drawStr((SCR_W - tw) / 2, bannerY + 7, a.text);
                u8g2.setDrawColor(1);
            } else {
                u8g2.drawRFrame(MARGIN, bannerY, SCR_W - 2 * MARGIN, bannerH, 2);
                u8g2.setFont(u8g2_font_helvB08_tr);
                int tw = u8g2.getStrWidth(a.text);
                int slide = (frameCounter / 2) % 8;
                int px = (SCR_W - tw) / 2 - (slide - 4);
                u8g2.drawStr(px, bannerY + 7, a.text);
            }
        } else {
            contentY += 2;
        }

        // Page content
        switch (oledPage) {
            case 0: { // VITALS
                int boxW = (SCR_W - 2 * MARGIN - GAP) / 2;
                int boxH = 26;
                char bpmStr[16], spo2Str[16];
                safeSprintf(bpmStr, sizeof(bpmStr), "%.0f", snap.bpm > 0.1f ? snap.bpm : 0.0f);
                safeSprintf(spo2Str, sizeof(spo2Str), "%.0f", snap.spo2 > 0.1f ? snap.spo2 : 0.0f);

                int bx = MARGIN;
                drawMetricBoxSafe(bx, contentY, boxW, boxH, "BPM", bpmStr, "");
                bx += boxW + GAP;
                drawMetricBoxSafe(bx, contentY, boxW, boxH, "SpO2", spo2Str, "%");

                // Waveform
                int waveTop = contentY + boxH + 2;
                int waveBottom = SCR_H - MARGIN - 1;
                int waveHeight = waveBottom - waveTop - 2;
                if (waveHeight < 6) waveHeight = 6;

                int32_t minVal = INT32_MAX, maxVal = INT32_MIN;
                int validCount = 0;
                for (int i = 0; i < DISP_WIDTH; i++) {
                    int32_t v = dispBuf[i];
                    if (v > 0) { validCount++; if (v < minVal) minVal = v; if (v > maxVal) maxVal = v; }
                }
                if (validCount == 0) { minVal = 0; maxVal = 50000; }
                if (minVal == maxVal) maxVal = minVal + 1;

                u8g2.drawHLine(MARGIN, waveBottom, SCR_W - 2 * MARGIN);
                int displayW = SCR_W - 2 * MARGIN;
                for (int x = 1; x < displayW; x++) {
                    int bufIdx0 = (idx + (x - 1)) % DISP_WIDTH;
                    int bufIdx1 = (idx + x) % DISP_WIDTH;
                    int32_t v0 = dispBuf[bufIdx0], v1 = dispBuf[bufIdx1];
                    if (v0 > 0 && v1 > 0) {
                        int y0 = waveBottom - (int)(((float)(v0 - minVal) / (float)(maxVal - minVal)) * waveHeight);
                        int y1 = waveBottom - (int)(((float)(v1 - minVal) / (float)(maxVal - minVal)) * waveHeight);
                        y0 = constrain(y0, waveTop, waveBottom);
                        y1 = constrain(y1, waveTop, waveBottom);
                        u8g2.drawLine(MARGIN + x - 1, y0, MARGIN + x, y1);
                    }
                }
                break;
            }

            case 1: { // MOTION
                int colX = MARGIN, colW = SCR_W - 2 * MARGIN, y = contentY + 2;
                char val[16];
                u8g2.setFont(u8g2_font_tom_thumb_4x6_tr);
                u8g2.drawStr(colX, y, "ORIENTATION");
                y += 8;
                safeSprintf(val, sizeof(val), "%.1f", snap.roll); drawLabelValueBox(colX, y, colW, "Roll", val, "\xB0"); y += 10;
                safeSprintf(val, sizeof(val), "%.1f", snap.pitch); drawLabelValueBox(colX, y, colW, "Pitch", val, "\xB0"); y += 10;
                safeSprintf(val, sizeof(val), "%.1f", snap.yaw); drawLabelValueBox(colX, y, colW, "Yaw", val, "\xB0"); y += 12;

                int evY = y, col = colX;
                int evW = (colW - 6) / 3;
                auto smallEvent = [&](const char* label, bool active) {
                    if (active) { u8g2.drawBox(col, evY - 6, evW - 2, 8); u8g2.setDrawColor(0); u8g2.drawStr(col + 2, evY, label); u8g2.setDrawColor(1); }
                    else { u8g2.drawFrame(col, evY - 6, evW - 2, 8); u8g2.drawStr(col + 2, evY, label); }
                    col += evW;
                };
                smallEvent("VIB", snap.vibration); smallEvent("FALL", snap.fall); smallEvent("SHOCK", snap.shock);

                y = evY + 12; col = colX;
                if (snap.freefall) { u8g2.drawFrame(col, y - 6, evW - 2, 8); u8g2.drawStr(col + 2, y, "FREEF"); } col += evW;
                if (snap.tilt) { u8g2.drawFrame(col, y - 6, evW - 2, 8); u8g2.drawStr(col + 2, y, "TILT"); }
                break;
            }

            case 2: { // ENVIRONMENT
                int colX = MARGIN, colW = SCR_W - 2 * MARGIN;
                int y = contentY + 6; // extra padding below header
                char v[20];
                if (!isnan(snap.tempBME)) {
                    safeSprintf(v, sizeof(v), "%.1f", snap.tempBME); drawLabelValueBox(colX, y, colW, "Temperature", v, "\xB0C"); y += 14;
                    safeSprintf(v, sizeof(v), "%.1f", snap.humBME); drawLabelValueBox(colX, y, colW, "Humidity", v, "%"); y += 14;
                    safeSprintf(v, sizeof(v), "%.0f", snap.pressBME); drawLabelValueBox(colX, y, colW, "Pressure", v, "hPa"); y += 14;
                } else {
                    u8g2.setFont(u8g2_font_helvB08_tr);
                    u8g2.drawStr(18, 34, "Sensors warming up...");
                }
                break;
            }

            case 3: { // AIR QUALITY
                int colX = MARGIN, colW = SCR_W - 2 * MARGIN, y = contentY + 2;
                char v[20];
                if (!isnan(snap.co2ppm)) {
                    safeSprintf(v, sizeof(v), "%.0f ppm", snap.co2ppm);
                    u8g2.setFont(u8g2_font_helvB08_tr);
                    int tw = u8g2.getStrWidth("CO2 / Air Quality");
                    u8g2.drawStr((SCR_W - tw) / 2, y + 8, "CO2 / Air Quality");

                    u8g2.setFont(u8g2_font_helvB10_tr);
                    tw = u8g2.getStrWidth(v);
                    u8g2.drawStr((SCR_W - tw) / 2, y + 24, v);

                    int barW = SCR_W - 2 * MARGIN - 20;
                    int fill = constrain((int)((snap.co2ppm / 2000.0f) * barW), 0, barW);
                    u8g2.drawFrame(MARGIN + 10, y + 30, barW, 6);
                    u8g2.drawBox(MARGIN + 11, y + 31, fill, 4);
                } else {
                    u8g2.setFont(u8g2_font_helvB08_tr);
                    u8g2.drawStr(20, 34, "Measuring air quality...");
                }
                break;
            }

            case 4: { // LOCATION
                int x = MARGIN, colW = SCR_W - 2 * MARGIN;
                int y = contentY + 6; // extra padding below header
                char val[32];
                if (snap.gpsOK) {
                    u8g2.setFont(u8g2_font_helvB10_tr); // slightly larger font
                    safeSprintf(val, sizeof(val), "%.6f", snap.lat); drawLabelValueBox(x, y, colW, "Latitude", val); y += 14;
                    safeSprintf(val, sizeof(val), "%.6f", snap.lon); drawLabelValueBox(x, y, colW, "Longitude", val); y += 14;
                    safeSprintf(val, sizeof(val), "%.1f", snap.alt); drawLabelValueBox(x, y, colW, "Altitude", val, "m"); y += 14;
                    safeSprintf(val, sizeof(val), "%.1f", snap.speed); drawLabelValueBox(x, y, colW, "Speed", val, "km/h");
                } else {
                    u8g2.setFont(u8g2_font_helvB08_tr);
                    u8g2.drawStr(18, 30, "Searching for");
                    u8g2.drawStr(25, 42, "GPS signal");
                    int dots = (frameCounter / 10) % 4;
                    for (int i = 0; i < dots; i++) u8g2.drawDisc(48 + i * 8, 50, 2);
                }
                break;
            }
        }

        u8g2.sendBuffer();
    }
}

void loop(){ 
  vTaskDelay(pdMS_TO_TICKS(1000)); 
}

