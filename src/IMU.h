#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stddef.h>

/*
  IMU.h - High quality Custom MPU6500/9250 IMU module (extended)
  - register dump
  - gyro bias calibration
  - raw read & conversion to g / dps
  - Madgwick AHRS (IMU-only)
  - IIR filters for gravity/dynamic separation
  - RMS vibration window
  - event detection: free-fall, shock, tilt (sliding-window), vibration, fall
  - persistent storage (NVS/Preferences) with version + CRC32 checksum
  - accel magnitude LPF to stabilize magnitude-driven thresholds
  - runtime setters for tilt params & persistent saving
*/

struct IMUConfig {
  uint8_t i2c_addr = 0x68;
  int calib_samples = 2000;
  float beta = 0.08f;        // Madgwick gain
  float lp_cutoff_hz = 5.0f; // lowpass cutoff for gravity estimate (Hz)
  int loop_hz = 200;         // target update Hz

  // thresholds (tunable)
  float freefall_thresh_g = 0.3f;
  float shock_thresh_g = 2.5f;
  float tilt_delta_deg = 20.0f;       // angle (deg) threshold for tilt detection
  float vibration_rms_thresh = 0.4f;

  // vibration window and tilt snap window
  int vib_window_samples = 100;       // window for RMS (~0.5s @ 200Hz)
  int tilt_snap_ms = 200;             // compare orientation to 200ms ago
  int tilt_confirm_count = 2;         // number of successive detections required to confirm tilt

  // accel magnitude smoother
  float accel_lpf_hz = 2.0f;          // lowpass for accel magnitude to stabilize freefall/vibration

  // version tag for stored config (update if you change stored layout)
  uint16_t cfg_version = 1;
};

struct ImuRaw {
  float ax_g, ay_g, az_g;   // acceleration in g
  float gx_dps, gy_dps, gz_dps; // gyro in deg/s
  float temp_c;
};

struct ImuEuler {
  float roll;   // degrees
  float pitch;  // degrees
  float yaw;    // degrees (drifting without magnetometer)
};

struct ImuEvents {
  bool freefall = false;
  bool shock = false;
  bool tilt = false;
  bool vibration = false;
  bool fall = false;
};

class IMU {
public:
  IMU(const IMUConfig &cfg = IMUConfig());
  ~IMU();

  bool begin();                     // init I2C and sensor, returns true if device present
  void dumpRegisters();             // print 0x00..0x7F to Serial

  bool calibrateGyro();             // must keep device still during this call
  bool update();                    // read sensors, update fusion and internal state; returns true if ok

  ImuRaw getRaw() const;
  ImuEuler getEuler() const;
  ImuEvents getEvents() const;
  float getVibrationRMS() const;

  void setBeta(float b);           // adjust Madgwick gain
  void setLPcutoffHz(float hz);    // adjust low-pass gravity cutoff

  // Tilt runtime setters (also can be persisted via savePersistentConfig)
  void setTiltDeltaDeg(float deg);
  void setTiltConfirmCount(int cnt);
  void setTiltSnapMs(int ms);
  void setAccelLpfHz(float hz);

  // Persistent config API (NVS/Preferences)
  // returns true if valid config was loaded (matching version + checksum)
  bool loadPersistentConfig();
  void savePersistentConfig();     // save current biases & thresholds (writes version + checksum)
  void resetPersistentConfig();    // clear saved config from NVS

private:
  IMUConfig cfg_;
  // gyro bias LSB
  float gbx_, gby_, gbz_;
  // quaternion state
  float q0_, q1_, q2_, q3_;
  // filtered gravity (IIR low-pass)
  float gx_lp_, gy_lp_, gz_lp_;
  // accel magnitude low-pass
  float acc_mag_lpf_; // smoothed acc magnitude (g)

  // small buffer for vibration RMS (circular)
  float *vib_buf_;
  int vib_idx_;
  int vib_count_;

  // quaternion sliding-window buffer (for tilt detection)
  float *q0_buf_;
  float *q1_buf_;
  float *q2_buf_;
  float *q3_buf_;
  unsigned long *q_ts_buf_;
  int q_buf_n_;
  int q_idx_;
  int q_count_;

  // tilt confirmation counter
  int tilt_confirm_seen_;

  // last micros for dt
  unsigned long last_micros_;
  // last Euler for delta detection
  float last_roll_, last_pitch_;
  // last computed values (public getters)
  ImuRaw last_raw_;
  ImuEuler last_euler_;
  ImuEvents last_events_;
  float last_vib_rms_;

  // helper low-level i2c
  bool i2cReadByte(uint8_t reg, uint8_t &out);
  bool i2cReadBlock(uint8_t reg, uint8_t *buf, uint8_t len);
  bool i2cWriteByte(uint8_t reg, uint8_t val);

  // math helpers
  void madgwickUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
  void quaternionToEuler();

  // processing helpers
  void pushVibSample(float sample);
  float computeVibRMS();
  void processEvents(float ax_g, float ay_g, float az_g, float gx_dps, float gy_dps, float gz_dps, float dt);

  // buffer helpers
  void allocBuffersForWindowSizes();
  void freeBuffers();

  // checksum helper
  uint32_t computeConfigCRC(const uint8_t *data, size_t len);

  // constants
  static constexpr float ACC_LSB_PER_G = 16384.0f;    // default ±2g
  static constexpr float GYRO_LSB_PER_DPS = 131.0f;   // default ±250 dps
};

#endif // IMU_H
