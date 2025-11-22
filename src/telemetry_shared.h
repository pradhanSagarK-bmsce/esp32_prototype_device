// telemetry_shared.h
#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Keep this in sync with your existing Telemetry struct fields.
// Replace the Telemetry definition in your main file with `#include "telemetry_shared.h"`
// and remove any duplicate Telemetry definition.

struct Telemetry {
  float bpm;
  float spo2;
  // IMU
  float roll, pitch, yaw;
  float ax, ay, az;
  float gx, gy, gz;
  bool freefall;
  bool shock;
  bool tilt;
  bool vibration;
  bool fall;
  bool smokeOrFire;
  bool urgent;
  // BME
  float tempBME, humBME, pressBME;
  // GPS
  double lat, lon, alt, speed;
  // MQ
  float co2ppm;
  // RAW & status
  int32_t lastIR;
  int32_t lastRED;
  bool maxOK;
  bool imuOK;
  bool bmeOK;
  bool gpsOK;

  Telemetry() {
    bpm = 0; spo2 = 0;
    roll = pitch = yaw = 0;
    ax = ay = az = 0;
    gx = gy = gz = 0;
    freefall = shock = tilt = vibration = fall = false;
    tempBME = humBME = pressBME = NAN;
    lat = lon = alt = speed = 0.0;
    co2ppm = NAN;
    lastIR = lastRED = 0;
    maxOK = imuOK = bmeOK = gpsOK = false;
  }
};

// NOTE: keep a single Telemetry instance in your main file (remove duplicates)
extern Telemetry telem;

// We'll rely on the same `dataMutex` you create in main to protect `telem`.
extern SemaphoreHandle_t dataMutex;
