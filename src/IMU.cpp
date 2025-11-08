#include "IMU.h"
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// Registers
static const uint8_t WHO_AM_I = 0x75;
static const uint8_t PWR_MGMT_1 = 0x6B;
static const uint8_t ACCEL_XOUT_H = 0x3B;
static const uint8_t ACCEL_CONFIG = 0x1C;
static const uint8_t GYRO_CONFIG = 0x1B;

// Preferences namespace and keys
static const char *PREFS_NAMESPACE = "imu_cfg_v1";
static const char *KEY_HAS_CAL = "hasCalib";
static const char *KEY_GBX = "gbx";
static const char *KEY_GBY = "gby";
static const char *KEY_GBZ = "gbz";
static const char *KEY_TH_FF = "th_ff";
static const char *KEY_TH_SH = "th_sh";
static const char *KEY_TH_VIB = "th_vib";
static const char *KEY_TH_TILT = "th_tilt";
static const char *KEY_BETA = "madg_bw";
static const char *KEY_LP_HZ = "lp_hz";
static const char *KEY_VIB_WINDOW = "vib_w";
static const char *KEY_TILT_SNAP_MS = "tilt_snap";
static const char *KEY_TILT_CONFIRM = "tilt_cnt";
static const char *KEY_ACCEL_LPF_HZ = "acc_lpf";

// version + checksum keys
static const char *KEY_CFG_VER = "cfg_ver";
static const char *KEY_CFG_CSUM = "cfg_csum";

// ---------------- constructor / destructor ----------------
IMU::IMU(const IMUConfig &cfg) : cfg_(cfg) {
  gbx_ = gby_ = gbz_ = 0.0f;
  q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
  gx_lp_ = gy_lp_ = gz_lp_ = 0.0f;
  acc_mag_lpf_ = 1.0f; // assume roughly gravity at start
  vib_buf_ = nullptr;
  vib_idx_ = vib_count_ = 0;

  q0_buf_ = q1_buf_ = q2_buf_ = q3_buf_ = nullptr;
  q_ts_buf_ = nullptr;
  q_buf_n_ = q_idx_ = q_count_ = 0;
  tilt_confirm_seen_ = 0;

  last_micros_ = micros();
  last_roll_ = last_pitch_ = 0.0f;
  last_vib_rms_ = 0.0f;

  allocBuffersForWindowSizes();
}

IMU::~IMU() {
  freeBuffers();
}

// allocate / free buffers (vibration and quaternion history)
void IMU::allocBuffersForWindowSizes() {
  // vib buffer
  if (vib_buf_) free(vib_buf_);
  int n_vib = max(1, cfg_.vib_window_samples);
  vib_buf_ = (float *)malloc(sizeof(float) * n_vib);
  if (vib_buf_) memset(vib_buf_, 0, sizeof(float) * n_vib);
  vib_idx_ = vib_count_ = 0;

  // quaternion buffer length based on tilt_snap_ms and loop_hz (ensure at least 2)
  if (q0_buf_) free(q0_buf_);
  if (q1_buf_) free(q1_buf_);
  if (q2_buf_) free(q2_buf_);
  if (q3_buf_) free(q3_buf_);
  if (q_ts_buf_) free(q_ts_buf_);

  int nsamples = max(2, (cfg_.loop_hz * max(1, cfg_.tilt_snap_ms)) / 1000);
  q_buf_n_ = nsamples + 4; // small slack
  q0_buf_ = (float *)malloc(sizeof(float) * q_buf_n_);
  q1_buf_ = (float *)malloc(sizeof(float) * q_buf_n_);
  q2_buf_ = (float *)malloc(sizeof(float) * q_buf_n_);
  q3_buf_ = (float *)malloc(sizeof(float) * q_buf_n_);
  q_ts_buf_ = (unsigned long *)malloc(sizeof(unsigned long) * q_buf_n_);
  if (q0_buf_) memset(q0_buf_, 0, sizeof(float) * q_buf_n_);
  if (q1_buf_) memset(q1_buf_, 0, sizeof(float) * q_buf_n_);
  if (q2_buf_) memset(q2_buf_, 0, sizeof(float) * q_buf_n_);
  if (q3_buf_) memset(q3_buf_, 0, sizeof(float) * q_buf_n_);
  if (q_ts_buf_) memset(q_ts_buf_, 0, sizeof(unsigned long) * q_buf_n_);
  q_idx_ = q_count_ = 0;
}

void IMU::freeBuffers() {
  if (vib_buf_) { free(vib_buf_); vib_buf_ = nullptr; }
  if (q0_buf_) { free(q0_buf_); q0_buf_ = nullptr; }
  if (q1_buf_) { free(q1_buf_); q1_buf_ = nullptr; }
  if (q2_buf_) { free(q2_buf_); q2_buf_ = nullptr; }
  if (q3_buf_) { free(q3_buf_); q3_buf_ = nullptr; }
  if (q_ts_buf_) { free(q_ts_buf_); q_ts_buf_ = nullptr; }
  q_buf_n_ = q_idx_ = q_count_ = 0;
}

// ---------------- begin / dump ----------------
bool IMU::begin() {
  Wire.begin();
  Wire.setClock(400000);
  // Wake up device
  if (!i2cWriteByte(PWR_MGMT_1, 0x01)) return false;
  delay(10);
  // optional set ranges to defaults
  i2cWriteByte(ACCEL_CONFIG, 0x00); // ±2g
  i2cWriteByte(GYRO_CONFIG,  0x00); // ±250 dps
  // read WHO_AM_I
  uint8_t who = 0xFF;
  if (!i2cReadByte(WHO_AM_I, who)) return false;
  Serial.printf("IMU: WHO_AM_I = 0x%02X\n", who);

  // Load saved config if present (biases + thresholds).
  if (loadPersistentConfig()) {
    Serial.println("IMU: Persistent config loaded and verified.");
  } else {
    Serial.println("IMU: No valid persistent config loaded (defaults active).");
  }

  // (re)alloc buffers after any loaded config
  allocBuffersForWindowSizes();

  return true;
}

void IMU::dumpRegisters() {
  Serial.println("---- IMU Register Dump 0x00..0x7F ----");
  for (uint8_t r = 0x00; r <= 0x7F; ++r) {
    uint8_t v;
    if (i2cReadByte(r, v)) {
      Serial.printf("0x%02X: 0x%02X\n", r, v);
    } else {
      Serial.printf("0x%02X: --\n", r);
    }
  }
  Serial.println("---- End Dump ----");
}

// ---------------- calibration ----------------
bool IMU::calibrateGyro() {
  Serial.println("IMU: calibrating gyro bias — keep device still");
  long sx = 0, sy = 0, sz = 0;
  uint8_t buf[14];
  int samples = cfg_.calib_samples;
  for (int i = 0; i < samples; ++i) {
    if (!i2cReadBlock(ACCEL_XOUT_H, buf, 14)) {
      Serial.println("IMU: I2C read error during calibration");
      return false;
    }
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];
    sx += gx; sy += gy; sz += gz;
    if ((i & 0x3F) == 0) delay(1); // yield occasionally
  }
  gbx_ = (float)sx / (float)samples;
  gby_ = (float)sy / (float)samples;
  gbz_ = (float)sz / (float)samples;

  Serial.printf("IMU: gyro bias (LSB) x=%.2f y=%.2f z=%.2f\n", gbx_, gby_, gbz_);

  // Save to persistent storage (includes version + CRC32)
  savePersistentConfig();

  return true;
}

// ---------------- update / conversion / filters ----------------
bool IMU::update() {
  // compute dt
  unsigned long now = micros();
  float dt = (now - last_micros_) * 1e-6f;
  if (dt <= 0) dt = 1.0f / (float)cfg_.loop_hz;
  last_micros_ = now;

  uint8_t buf[14];
  if (!i2cReadBlock(ACCEL_XOUT_H, buf, 14)) return false;

  int16_t AcX = (buf[0] << 8) | buf[1];
  int16_t AcY = (buf[2] << 8) | buf[3];
  int16_t AcZ = (buf[4] << 8) | buf[5];
  int16_t Tmp = (buf[6] << 8) | buf[7];
  int16_t GyX = (buf[8] << 8) | buf[9];
  int16_t GyY = (buf[10] << 8) | buf[11];
  int16_t GyZ = (buf[12] << 8) | buf[13];

  // convert to physical units
  float ax_g = (float)AcX / ACC_LSB_PER_G;
  float ay_g = (float)AcY / ACC_LSB_PER_G;
  float az_g = (float)AcZ / ACC_LSB_PER_G;
  float gx_dps = ((float)GyX - gbx_) / GYRO_LSB_PER_DPS;
  float gy_dps = ((float)GyY - gby_) / GYRO_LSB_PER_DPS;
  float gz_dps = ((float)GyZ - gbz_) / GYRO_LSB_PER_DPS;
  float tempC = (float)Tmp / 340.0f + 36.53f;

  // low-pass filter gravity estimate (IIR): alpha = dt/(dt + tau), tau = 1/(2*pi*fc)
  float tau = 1.0f / (2.0f * (float)M_PI * cfg_.lp_cutoff_hz);
  float alpha = dt / (dt + tau);
  gx_lp_ = (1.0f - alpha) * gx_lp_ + alpha * ax_g;
  gy_lp_ = (1.0f - alpha) * gy_lp_ + alpha * ay_g;
  gz_lp_ = (1.0f - alpha) * gz_lp_ + alpha * az_g;

  // accel magnitude low-pass (complementary-like single-pole)
  float accMag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float tau_acc = 1.0f / (2.0f * (float)M_PI * max(0.001f, cfg_.accel_lpf_hz));
  float alpha_acc = dt / (dt + tau_acc);
  acc_mag_lpf_ = (1.0f - alpha_acc) * acc_mag_lpf_ + alpha_acc * accMag;

  // dynamic accel = accel - gravity estimate (not used for tilt detection)
  float dax = ax_g - gx_lp_;
  float day = ay_g - gy_lp_;
  float daz = az_g - gz_lp_;

  // push vibration sample (use magnitude deviation from 1g) (use filtered magnitude for stability)
  pushVibSample(fabsf(acc_mag_lpf_ - 1.0f));
  float vib_rms = computeVibRMS();

  // update last_raw
  last_raw_.ax_g = ax_g; last_raw_.ay_g = ay_g; last_raw_.az_g = az_g;
  last_raw_.gx_dps = gx_dps; last_raw_.gy_dps = gy_dps; last_raw_.gz_dps = gz_dps;
  last_raw_.temp_c = tempC;

  // run Madgwick IMU update: uses gyro in rad/s and accel in g
  madgwickUpdateIMU(gx_dps * (M_PI/180.0f), gy_dps * (M_PI/180.0f), gz_dps * (M_PI/180.0f),
                    ax_g, ay_g, az_g, dt);

  // store quaternion sample (millis timestamp)
  unsigned long now_ms = millis();
  if (q_buf_n_ > 1) {
    q0_buf_[q_idx_] = q0_;
    q1_buf_[q_idx_] = q1_;
    q2_buf_[q_idx_] = q2_;
    q3_buf_[q_idx_] = q3_;
    q_ts_buf_[q_idx_] = now_ms;
    q_idx_ = (q_idx_ + 1) % q_buf_n_;
    if (q_count_ < q_buf_n_) q_count_++;
  }

  // get Euler
  quaternionToEuler();

  // process events (use acc_mag_lpf_ for magnitude-driven events)
  processEvents(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, dt);

  last_vib_rms_ = vib_rms;
  last_euler_.roll = last_roll_;
  last_euler_.pitch = last_pitch_;
  // yaw compute from quaternion
  float siny_cosp = 2.0f * (q0_ * q3_ + q1_ * q2_);
  float cosy_cosp = 1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_);
  last_euler_.yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;

  return true;
}

// getters
ImuRaw IMU::getRaw() const { return last_raw_; }
ImuEuler IMU::getEuler() const { return last_euler_; }
ImuEvents IMU::getEvents() const { return last_events_; }
float IMU::getVibrationRMS() const { return last_vib_rms_; }

void IMU::setBeta(float b) { cfg_.beta = b; }
void IMU::setLPcutoffHz(float hz) { cfg_.lp_cutoff_hz = hz; }

// runtime setters for tilt and accel LPF
void IMU::setTiltDeltaDeg(float deg) {
  cfg_.tilt_delta_deg = deg;
}
void IMU::setTiltConfirmCount(int cnt) {
  cfg_.tilt_confirm_count = max(1, cnt);
}
void IMU::setTiltSnapMs(int ms) {
  cfg_.tilt_snap_ms = max(10, ms);
  // reallocate quaternion buffer accordingly
  allocBuffersForWindowSizes();
}
void IMU::setAccelLpfHz(float hz) {
  cfg_.accel_lpf_hz = max(0.01f, hz);
}

// ---------------- Persistent Configuration Storage ----------------

uint32_t IMU::computeConfigCRC(const uint8_t *data, size_t len) {
  // standard CRC32 (polynomial 0xEDB88320)
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    uint8_t byte = data[i];
    crc ^= (uint32_t)byte;
    for (int j = 0; j < 8; ++j) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

bool IMU::loadPersistentConfig() {
  Preferences prefs;
  prefs.begin(PREFS_NAMESPACE, true); // read-only

  // Read stored version first
  uint32_t stored_ver = prefs.getUInt(KEY_CFG_VER, 0);
  if (stored_ver == 0) {
    prefs.end();
    return false;
  }

  // If versions mismatch -> don't load (safe upgrade)
  if (stored_ver != (uint32_t)cfg_.cfg_version) {
    Serial.printf("IMU: stored cfg version %u != expected %u -> skipping load\n", stored_ver, (uint32_t)cfg_.cfg_version);
    prefs.end();
    return false;
  }

  // read stored checksum
  uint32_t stored_crc = prefs.getUInt(KEY_CFG_CSUM, 0);

  // Build buffer to compute CRC using exact order; MUST match savePersistentConfig packing order
  const size_t BUF_SZ = sizeof(float)*11 + sizeof(int)*2 + sizeof(uint32_t);
  uint8_t buffer[BUF_SZ];
  size_t offset = 0;
  auto packFloat = [&](float v){ memcpy(buffer + offset, &v, sizeof(float)); offset += sizeof(float); };
  auto packInt = [&](int v){ memcpy(buffer + offset, &v, sizeof(int)); offset += sizeof(int); };
  auto packUInt32 = [&](uint32_t v){ memcpy(buffer + offset, &v, sizeof(uint32_t)); offset += sizeof(uint32_t); };

  // read values (fallback to defaults in cfg_ / runtime)
  float r_gbx = prefs.getFloat(KEY_GBX, gbx_);
  float r_gby = prefs.getFloat(KEY_GBY, gby_);
  float r_gbz = prefs.getFloat(KEY_GBZ, gbz_);
  float r_th_ff = prefs.getFloat(KEY_TH_FF, cfg_.freefall_thresh_g);
  float r_th_sh = prefs.getFloat(KEY_TH_SH, cfg_.shock_thresh_g);
  float r_th_vib = prefs.getFloat(KEY_TH_VIB, cfg_.vibration_rms_thresh);
  float r_th_tilt = prefs.getFloat(KEY_TH_TILT, cfg_.tilt_delta_deg);
  float r_beta = prefs.getFloat(KEY_BETA, cfg_.beta);
  float r_lp_hz = prefs.getFloat(KEY_LP_HZ, cfg_.lp_cutoff_hz);
  int r_vib_w = prefs.getInt(KEY_VIB_WINDOW, cfg_.vib_window_samples);
  int r_tilt_snap = prefs.getInt(KEY_TILT_SNAP_MS, cfg_.tilt_snap_ms);
  int r_tilt_cnt = prefs.getInt(KEY_TILT_CONFIRM, cfg_.tilt_confirm_count);
  float r_acc_lpf = prefs.getFloat(KEY_ACCEL_LPF_HZ, cfg_.accel_lpf_hz);

  // pack in same order
  packFloat(r_gbx);
  packFloat(r_gby);
  packFloat(r_gbz);
  packFloat(r_th_ff);
  packFloat(r_th_sh);
  packFloat(r_th_vib);
  packFloat(r_th_tilt);
  packFloat(r_beta);
  packFloat(r_lp_hz);
  packInt(r_vib_w);
  packInt(r_tilt_snap);
  packInt(r_tilt_cnt);
  packFloat(r_acc_lpf);
  packUInt32(stored_ver);

  // compute crc
  uint32_t crc = computeConfigCRC(buffer, offset);

  if (crc != stored_crc) {
    Serial.printf("IMU: config CRC mismatch (stored 0x%08X != calc 0x%08X) -> skip load\n", stored_crc, crc);
    prefs.end();
    return false;
  }

  // commit loaded values to runtime
  gbx_ = r_gbx; gby_ = r_gby; gbz_ = r_gbz;
  cfg_.freefall_thresh_g = r_th_ff;
  cfg_.shock_thresh_g = r_th_sh;
  cfg_.vibration_rms_thresh = r_th_vib;
  cfg_.tilt_delta_deg = r_th_tilt;
  cfg_.beta = r_beta;
  cfg_.lp_cutoff_hz = r_lp_hz;
  cfg_.vib_window_samples = r_vib_w;
  cfg_.tilt_snap_ms = r_tilt_snap;
  cfg_.tilt_confirm_count = r_tilt_cnt;
  cfg_.accel_lpf_hz = r_acc_lpf;

  prefs.end();

  // reallocate buffers if sizes changed
  allocBuffersForWindowSizes();

  Serial.println("IMU: persistent config loaded and CRC verified.");
  return true;
}

void IMU::savePersistentConfig() {
  Preferences prefs;
  prefs.begin(PREFS_NAMESPACE, false); // write mode

  // We'll compute CRC over exact sequence used in loadPersistentConfig
  const size_t BUF_SZ = sizeof(float)*11 + sizeof(int)*2 + sizeof(uint32_t);
  uint8_t buffer[BUF_SZ];
  size_t offset = 0;
  auto packFloat = [&](float v){ memcpy(buffer + offset, &v, sizeof(float)); offset += sizeof(float); };
  auto packInt = [&](int v){ memcpy(buffer + offset, &v, sizeof(int)); offset += sizeof(int); };
  auto packUInt32 = [&](uint32_t v){ memcpy(buffer + offset, &v, sizeof(uint32_t)); offset += sizeof(uint32_t); };

  packFloat(gbx_);
  packFloat(gby_);
  packFloat(gbz_);
  packFloat(cfg_.freefall_thresh_g);
  packFloat(cfg_.shock_thresh_g);
  packFloat(cfg_.vibration_rms_thresh);
  packFloat(cfg_.tilt_delta_deg);
  packFloat(cfg_.beta);
  packFloat(cfg_.lp_cutoff_hz);
  packInt(cfg_.vib_window_samples);
  packInt(cfg_.tilt_snap_ms);
  packInt(cfg_.tilt_confirm_count);
  packFloat(cfg_.accel_lpf_hz);
  packUInt32((uint32_t)cfg_.cfg_version);

  uint32_t crc = computeConfigCRC(buffer, offset);

  // store values individually (so they are human-readable via NVS)
  prefs.putBool(KEY_HAS_CAL, true);
  prefs.putFloat(KEY_GBX, gbx_);
  prefs.putFloat(KEY_GBY, gby_);
  prefs.putFloat(KEY_GBZ, gbz_);

  prefs.putFloat(KEY_TH_FF, cfg_.freefall_thresh_g);
  prefs.putFloat(KEY_TH_SH, cfg_.shock_thresh_g);
  prefs.putFloat(KEY_TH_VIB, cfg_.vibration_rms_thresh);
  prefs.putFloat(KEY_TH_TILT, cfg_.tilt_delta_deg);
  prefs.putFloat(KEY_BETA, cfg_.beta);
  prefs.putFloat(KEY_LP_HZ, cfg_.lp_cutoff_hz);
  prefs.putInt(KEY_VIB_WINDOW, cfg_.vib_window_samples);

  prefs.putInt(KEY_TILT_SNAP_MS, cfg_.tilt_snap_ms);
  prefs.putInt(KEY_TILT_CONFIRM, cfg_.tilt_confirm_count);
  prefs.putFloat(KEY_ACCEL_LPF_HZ, cfg_.accel_lpf_hz);

  prefs.putUInt(KEY_CFG_VER, (uint32_t)cfg_.cfg_version);
  prefs.putUInt(KEY_CFG_CSUM, crc);

  prefs.end();
  Serial.printf("IMU: Persistent config saved (ver=%u crc=0x%08X)\n", (uint32_t)cfg_.cfg_version, crc);
}

void IMU::resetPersistentConfig() {
  Preferences prefs;
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.clear();
  prefs.end();
  Serial.println("IMU: Persistent config cleared.");
}

// ----------------- low-level i2c -----------------
bool IMU::i2cReadByte(uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(cfg_.i2c_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((int)cfg_.i2c_addr, 1);
  if (!Wire.available()) return false;
  out = Wire.read();
  return true;
}

bool IMU::i2cReadBlock(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(cfg_.i2c_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((int)cfg_.i2c_addr, (int)len);
  for (uint8_t i = 0; i < len; ++i) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }
  return true;
}

bool IMU::i2cWriteByte(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(cfg_.i2c_addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// --------------- Madgwick (IMU-only) ---------------
void IMU::madgwickUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  // Use cfg_.beta as gain
  float beta = cfg_.beta;

  // normalize accel
  float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm == 0.0f) return;
  ax /= norm; ay /= norm; az /= norm;

  // shorthand
  float q0 = q0_, q1 = q1_, q2 = q2_, q3 = q3_;

  float _2q0 = 2.0f*q0;
  float _2q1 = 2.0f*q1;
  float _2q2 = 2.0f*q2;
  float _2q3 = 2.0f*q3;
  float _4q0 = 4.0f*q0;
  float _4q1 = 4.0f*q1;
  float _4q2 = 4.0f*q2;
  float _8q1 = 8.0f*q1;
  float _8q2 = 8.0f*q2;
  float q0q0 = q0*q0;
  float q1q1 = q1*q1;
  float q2q2 = q2*q2;
  float q3q3 = q3*q3;

  // gradient step (from Madgwick)
  float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
  float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
  float s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
  float s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;
  norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
  if (norm == 0.0f) return;
  s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm;

  // q derivative
  float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz) - beta * s0;
  float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy) - beta * s1;
  float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx) - beta * s2;
  float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx) - beta * s3;

  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  // normalize quaternion
  norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0_ = q0 / norm;
  q1_ = q1 / norm;
  q2_ = q2 / norm;
  q3_ = q3 / norm;
}

void IMU::quaternionToEuler() {
  // roll
  float sinr_cosp = 2.0f * (q0_*q1_ + q2_*q3_);
  float cosr_cosp = 1.0f - 2.0f * (q1_*q1_ + q2_*q2_);
  float roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
  // pitch
  float sinp = 2.0f * (q0_*q2_ - q3_*q1_);
  float pitch;
  if (fabsf(sinp) >= 1.0f) pitch = copysignf(90.0f, sinp);
  else pitch = asinf(sinp) * 180.0f / M_PI;
  last_roll_ = roll;
  last_pitch_ = pitch;
}

// vibration buffer
void IMU::pushVibSample(float sample) {
  int n = max(1, cfg_.vib_window_samples);
  vib_buf_[vib_idx_] = sample;
  vib_idx_ = (vib_idx_ + 1) % n;
  if (vib_count_ < n) vib_count_++;
}

float IMU::computeVibRMS() {
  int n = vib_count_;
  if (n == 0) return 0.0f;
  float sum = 0.0f;
  for (int i = 0; i < n; ++i) sum += vib_buf_[i] * vib_buf_[i];
  return sqrtf(sum / (float)n);
}

// compute angle difference (deg) between current quaternion and one ~snap_ms ago
static float quaternionAngleDeg(const float q0a, const float q1a, const float q2a, const float q3a,
                                const float q0b, const float q1b, const float q2b, const float q3b) {
  // dot product
  float dot = q0a*q0b + q1a*q1b + q2a*q2b + q3a*q3b;
  if (dot > 1.0f) dot = 1.0f;
  if (dot < -1.0f) dot = -1.0f;
  // angle = 2 * acos(|dot|)
  float ang = 2.0f * acosf(fabsf(dot));
  return ang * 180.0f / M_PI;
}

// ---------------- events / tilt detection ----------------
void IMU::processEvents(float ax_g, float ay_g, float az_g,
                        float gx_dps, float gy_dps, float gz_dps, float dt) {
  // recompute useful metrics
  float accMag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float pitch = last_pitch_;
  float roll = last_roll_;

  float vib_rms = computeVibRMS();

  // base classification using filtered magnitude for stability
  ImuEvents e;
  e.freefall = (acc_mag_lpf_ < cfg_.freefall_thresh_g);
  e.shock    = (acc_mag_lpf_ > cfg_.shock_thresh_g) ||
               (fabsf(gx_dps) > 300.0f) || (fabsf(gy_dps) > 300.0f) || (fabsf(gz_dps) > 300.0f);
  e.vibration = (vib_rms > cfg_.vibration_rms_thresh);

  // ---- sliding-window quaternion tilt detection ----
  // Find quaternion sample closest to now - snap_ms
  bool tilt_detected = false;
  if (q_count_ >= 2 && q_buf_n_ > 1) {
    unsigned long target_ms = millis() - (unsigned long)cfg_.tilt_snap_ms;
    // search back in buffer for entry <= target_ms
    int found = -1;
    // iterate last q_count_ entries
    for (int i = 0; i < q_count_; ++i) {
      int idx = (q_idx_ - 1 - i + q_buf_n_) % q_buf_n_; // i-th previous
      unsigned long ts = q_ts_buf_[idx];
      if (ts <= target_ms) {
        found = idx;
        break;
      }
    }
    if (found != -1) {
      float q0_old = q0_buf_[found];
      float q1_old = q1_buf_[found];
      float q2_old = q2_buf_[found];
      float q3_old = q3_buf_[found];
      float angle_deg = quaternionAngleDeg(q0_, q1_, q2_, q3_, q0_old, q1_old, q2_old, q3_old);
      if (angle_deg > cfg_.tilt_delta_deg) tilt_detected = true;
    } else {
      // If not enough history older than snap_ms, optionally compare to oldest stored sample
      int oldest = (q_idx_ - q_count_ + q_buf_n_) % q_buf_n_;
      float q0_old = q0_buf_[oldest];
      float q1_old = q1_buf_[oldest];
      float q2_old = q2_buf_[oldest];
      float q3_old = q3_buf_[oldest];
      float angle_deg = quaternionAngleDeg(q0_, q1_, q2_, q3_, q0_old, q1_old, q2_old, q3_old);
      if (angle_deg > cfg_.tilt_delta_deg) tilt_detected = true;
    }
  }

  // confirmation counter logic
  if (tilt_detected) {
    tilt_confirm_seen_++;
    if (tilt_confirm_seen_ >= cfg_.tilt_confirm_count) {
      e.tilt = true;
      // keep tilt_confirm_seen_ at cap so it doesn't overflow
      tilt_confirm_seen_ = cfg_.tilt_confirm_count;
    } else {
      e.tilt = false;
    }
  } else {
    // decay counter toward zero
    if (tilt_confirm_seen_ > 0) tilt_confirm_seen_--;
    e.tilt = false;
  }

  // fall event: freefall followed by shock within short window
  static unsigned long freefall_time = 0;
  if (e.freefall) freefall_time = micros();
  if (freefall_time != 0) {
    if (e.shock && (micros() - freefall_time) < 1000000UL) {
      e.fall = true;
      freefall_time = 0;
    } else if ((micros() - freefall_time) > 1000000UL) {
      // timeout
      freefall_time = 0;
    }
  }

  // store events
  last_events_ = e;
  last_vib_rms_ = vib_rms;
}



