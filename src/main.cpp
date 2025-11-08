#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "IMU.h"
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include "MAX30105.h"
#include "PulseAlgorithm.h"
#include <U8g2lib.h>

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

// ----------------- Devices -----------------
Adafruit_BME280 bme;
TwoWire I2CBME = TwoWire(1);  // BME280 on separate I2C
IMU imu;
HardwareSerial GPSs(2);
TinyGPSPlus gps;
MAX30105 particleSensor;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ----------------- RTOS resources -----------------
SemaphoreHandle_t i2cMutex = nullptr;
SemaphoreHandle_t dataMutex = nullptr;

// ----------------- Telemetry (shared) -----------------
struct Telemetry {
  float bpm = 0.0f;
  float spo2 = 0.0f;
  // IMU
  float roll=0, pitch=0, yaw=0;
  float ax=0, ay=0, az=0;
  float gx=0, gy=0, gz=0;
  float imuTemp = 0;
  // BME
  float tempBME = NAN, humBME = NAN, pressBME = NAN;
  // GPS
  double lat = 0.0, lon = 0.0, alt = 0.0, speed = 0.0;
  // MQ
  float co2ppm = NAN;
  // Raw latest IR/RED for display
  int32_t lastIR = 0;
  int32_t lastRED = 0;
  bool maxOK = false;
  bool imuOK = false;
  bool bmeOK = false;
  bool gpsOK = false;
} telem;

// Display state
static uint8_t oledPage = 0;

// ----------------- Task parameters -----------------
constexpr TickType_t MAX_TASK_DELAY = pdMS_TO_TICKS(1);
constexpr int STACK_SMALL = 4096;
constexpr int STACK_MED = 8192;

/* Task prototypes */
void maxTask(void* pv);
void imuTask(void* pv);
void gpsTask(void* pv);
void bmeTask(void* pv);
void mqTask(void* pv);
void calcTask(void* pv);
void oledTask(void* pv);

// ----------------- Utility helpers -----------------
void safeI2cBegin() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
}

bool safeParticleBegin() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    bool ok = particleSensor.begin(Wire, I2C_SPEED_STANDARD);
    xSemaphoreGive(i2cMutex);
    return ok;
  }
  return false;
}

void safeParticleSetup() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    particleSensor.setup(50, 4, 2, 100, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x24);
    particleSensor.setPulseAmplitudeIR(0x24);
    xSemaphoreGive(i2cMutex);
  }
}

// ----------------- setup -----------------
void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println("\n=== Multitask Sensor Hub (FreeRTOS) ===");

  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  if (!i2cMutex || !dataMutex) {
    Serial.println("Failed to create mutexes!");
    while(1) delay(1000);
  }

  // Main I2C for MAX30105/IMU
  safeI2cBegin();

  // BME280 on separate I2C
  I2CBME.begin(BME_SDA, BME_SCL, 400000);

  // Initialize devices
  if (!safeParticleBegin()) {
    Serial.println("MAX30105 not found at init (will retry in task).");
    telem.maxOK = false;
  } else {
    safeParticleSetup();
    resetPulseBuffers();
    telem.maxOK = true;
    Serial.println("MAX30105 init OK");
  }

  // BME280
  bool okBME = bme.begin(BME_ADDR, &I2CBME);
  telem.bmeOK = okBME;
  Serial.println(okBME ? "BME280 init OK" : "BME280 not found");

  // IMU
  imu.begin();
  imu.calibrateGyro();
  telem.imuOK = true;
  Serial.println("IMU init done");

  // GPS UART
  GPSs.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS UART started");

  // OLED
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(6,30,"Booting...");
  u8g2.sendBuffer();

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

  if (!telem.maxOK) {
    if (safeParticleBegin()) {
      safeParticleSetup();
      resetPulseBuffers();
      telem.maxOK = true;
      Serial.println("MAX30105 init (delayed) OK");
    } else {
      Serial.println("MAX30105 init delayed failed; task will keep trying.");
    }
  }

  for (;;) {
    vTaskDelayUntil(&lastWake, samplePeriod);
    int32_t ir=0, red=0;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      ir = particleSensor.getIR();
      red = particleSensor.getRed();
      xSemaphoreGive(i2cMutex);
    } else continue;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
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
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      ok = imu.update();
      xSemaphoreGive(i2cMutex);
    } else ok = imu.update();

    if (ok) {
      ImuEuler e = imu.getEuler();
      ImuRaw r = imu.getRaw();
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        telem.roll = e.roll; telem.pitch = e.pitch; telem.yaw = e.yaw;
        telem.ax = r.ax_g; telem.ay = r.ay_g; telem.az = r.az_g;
        telem.gx = r.gx_dps; telem.gy = r.gy_dps; telem.gz = r.gz_dps;
        telem.imuTemp = r.temp_c;
        telem.imuOK = true;
        xSemaphoreGive(dataMutex);
      }
    } else telem.imuOK = false;
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

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      telem.bpm = newBPM;
      telem.spo2 = newSpO2;
      xSemaphoreGive(dataMutex);
    }
    Serial.printf("Calc: BPM=%.1f SpO2=%.1f (bufferFull=%d)\n", newBPM, newSpO2, bufferFull?1:0);
  }
}

// ----------------- GPS Task -----------------
void gpsTask(void* pv) {
  const TickType_t propagatePeriod = pdMS_TO_TICKS(10000);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    while (GPSs.available()) gps.encode((char)GPSs.read());

    if ((xTaskGetTickCount() - lastWake) >= propagatePeriod) {
      lastWake = xTaskGetTickCount();
      if (gps.location.isValid() && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        telem.lat = gps.location.lat();
        telem.lon = gps.location.lng();
        telem.alt = gps.altitude.meters();
        telem.speed = gps.speed.kmph();
        telem.gpsOK = true;
        xSemaphoreGive(dataMutex);
        Serial.printf("GPS: lat=%.6f lon=%.6f alt=%.1f speed=%.2f\n", telem.lat, telem.lon, telem.alt, telem.speed);
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
    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0F;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      telem.tempBME = t; telem.humBME = h; telem.pressBME = p; telem.bmeOK = true;
      xSemaphoreGive(dataMutex);
    }
    Serial.printf("BME: T=%.2f H=%.2f P=%.2f\n", t, h, p);
  }
}

// ----------------- MQ Task -----------------
void mqTask(void* pv) {
  const TickType_t period = pdMS_TO_TICKS(3500);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    int raw = analogRead(MQ135_PIN);
    float voltage = raw * (3.3f / 4095.0f);
    float co2 = 116.6020682f * pow((voltage / 3.3f), -2.769034857f);

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      telem.co2ppm = co2;
      xSemaphoreGive(dataMutex);
    }
    Serial.printf("MQ135: CO2=%.1f ppm (ADC=%d)\n", co2, raw);
  }
}

// ----------------- OLED Task -----------------
void oledTask(void* pv) {
  const TickType_t period = pdMS_TO_TICKS(200);
  TickType_t lastWake = xTaskGetTickCount();

  const int DISP = 128;
  static int32_t dispBuf[DISP];
  static int dispIdx = 0;

  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    Telemetry snap;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      snap = telem;
      xSemaphoreGive(dataMutex);
    }

    dispBuf[dispIdx] = snap.lastIR>0 ? snap.lastIR : 0;
    dispIdx = (dispIdx+1) % DISP;

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    switch (oledPage) {
      case 0: {
        char s[32];
        snprintf(s, sizeof(s), "BPM: %.0f", snap.bpm>0.1f ? snap.bpm : 0);
        u8g2.drawStr(2,12,s);
        snprintf(s,sizeof(s),"SpO2: %.0f%%", snap.spo2>0.1f ? snap.spo2 : 0);
        u8g2.drawStr(70,12,s);
        break;
      }
      case 1: {
        char s[64];
        snprintf(s,sizeof(s),"R:%.1f P:%.1f Y:%.1f", snap.roll, snap.pitch, snap.yaw);
        u8g2.drawStr(2,16,s);
        break;
      }
      case 2: {
        char s[48];
        if (!isnan(snap.tempBME)) {
          snprintf(s,sizeof(s),"T:%.1fC H:%.1f%%", snap.tempBME, snap.humBME);
          u8g2.drawStr(2,16,s);
          snprintf(s,sizeof(s),"P:%.1fhPa", snap.pressBME);
          u8g2.drawStr(2,32,s);
        }
        if (!isnan(snap.co2ppm)) {
          snprintf(s,sizeof(s),"CO2: %.0f ppm", snap.co2ppm);
          u8g2.drawStr(2,48,s);
        }
        break;
      }
      case 3: {
        if (snap.gpsOK) {
          char s[64];
          snprintf(s,sizeof(s),"Lat: %.6f", snap.lat);
          u8g2.drawStr(2,12,s);
          snprintf(s,sizeof(s),"Lon: %.6f", snap.lon);
          u8g2.drawStr(2,28,s);
        } else {
          u8g2.drawStr(10,24,"GPS: waiting for fix...");
        }
        break;
      }
    }

    oledPage = (oledPage + 1) % 4;
    u8g2.sendBuffer();
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
