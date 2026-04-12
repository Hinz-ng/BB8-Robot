// module:  state_estimator.h
// layer:   2 — sensor fusion; no hardware I/O
// purpose: complementary filter — fuses gyro integration with accel tilt
//          correction to produce pitch and roll angle estimates
// inputs:  RawIMUData (from imu.h) — must already have gyro bias removed
// outputs: IMUState struct (pitch_rad, roll_rad, pitch_deg, roll_deg, valid)
// deps:    imu.h, project_wide_defs.h
// date:    2025
//
// Axis convention (VERIFY ON HARDWARE before trusting output):
//   Assumes IMU mounted so that at rest on a flat surface:
//     accel_z ≈ +9.81 m/s²  (Z points up)
//     accel_x ≈  0           (X points forward)
//     accel_y ≈  0           (Y points left)
//   If your mounting differs, update accelPitch() and accelRoll() accordingly.
//
// Filter:
//   pitch = CF_ALPHA * (pitch + gyro_y_rads * DT_S)
//         + (1 - CF_ALPHA) * atan2(-ax, sqrt(ay² + az²))
//   roll  = CF_ALPHA * (roll  + gyro_x_rads * DT_S)
//         + (1 - CF_ALPHA) * atan2(ay, az)
//
//   CF_ALPHA and IMU_LOOP_HZ come from project_wide_defs.h.
//   DT_S is derived from IMU_LOOP_HZ — do not hardcode it here.

#pragma once
#include "imu.h"
#include "project_wide_defs.h"
#include <cmath>

struct IMUState {
    float pitch_rad;  // rad — positive = nose up (see axis convention above)
    float roll_rad;   // rad — positive = right side down
    float pitch_deg;  // deg — convenience copy; same sign convention
    float roll_deg;   // deg
    bool  valid;      // false if input RawIMUData was invalid or filter not yet seeded
};

class StateEstimator {
public:
    // update() — call every IMU tick (at IMU_LOOP_HZ)
    // First call self-initialises from accel (no gyro integration on tick 0).
    // Returns invalid IMUState if raw.valid is false.
    IMUState update(const RawIMUData& raw);

    const IMUState& getState() const { return _state; }

    // reset() — forces re-initialisation from accel on the next update() call.
    // Call if the robot is known to have been stationary (e.g. after a hard
    // reset) to eliminate accumulated drift from a bad initial condition.
    void reset() { _initialized = false; }

private:
    IMUState _state{};
    bool     _initialized{false};

    // dt = 1 / IMU_LOOP_HZ — recalculate if loop rate changes.
    // Defined here so it stays in sync with the loop rate automatically.
    static constexpr float DT_S = 1.0f / static_cast<float>(IMU_LOOP_HZ);

    // Accel-derived tilt angles — used for filter correction term.
    // These are noisy but drift-free; the filter weights them at (1 - CF_ALPHA).
    static float accelPitch(const RawIMUData& d);
    static float accelRoll(const RawIMUData& d);
};