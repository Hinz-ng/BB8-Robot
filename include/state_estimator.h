// module:  state_estimator.h
// layer:   2 — sensor fusion; no hardware I/O
// purpose: complementary filter — fuses gyro integration with accel tilt
//          correction to produce pitch and roll angle estimates
// inputs:  RawIMUData (from imu.h) — must already have gyro bias removed
// outputs: IMUState struct (pitch_rad, roll_rad, pitch_deg, roll_deg,
//          pitch_rate_rads, roll_rate_rads, valid)
// deps:    imu.h, project_wide_defs.h
// date:    2025
//
// Mounting:
//   Raw sensor axes are remapped to a logical robot frame (forward/left/up)
//   by remapAxes() before any filter math runs. The filter formulas are
//   mounting-agnostic. Select mounting in project_wide_defs.h:
//     IMU_MOUNTING = IMUMounting::Z_FORWARD_Y_UP   (Z toward nose, Y up)
//     IMU_MOUNTING = IMUMounting::Z_FORWARD_X_UP   (Z toward nose, X up)
//     IMU_MOUNTING = IMUMounting::STANDARD          (Z up, X forward)
//
//   To verify your mounting: hold robot still on a flat surface and check
//   which accel axis reads ~+9.81. That axis is your "up" axis.
//
// Filter:
//   pitch = CF_ALPHA * (pitch + g_pitch * DT_S)
//         + (1 - CF_ALPHA) * atan2(-a_fwd, sqrt(a_left² + a_up²))
//   roll  = CF_ALPHA * (roll  + g_roll  * DT_S)
//         + (1 - CF_ALPHA) * atan2(a_left, a_up)
//
//   CF_ALPHA and IMU_LOOP_HZ come from project_wide_defs.h.

#pragma once
#include "imu.h"
#include "project_wide_defs.h"
#include <cmath>

struct IMUState {
    float pitch_rad;       // rad — positive = nose up
    float roll_rad;        // rad — positive = right side down
    float pitch_deg;       // deg — convenience copy for telemetry + PD inputs
    float roll_deg;        // deg
    float pitch_rate_rads; // rad/s — gyro in pitch axis (logical frame), bias-corrected
                           //         use this for Kd term; do NOT differentiate pitch_deg
    float roll_rate_rads;  // rad/s — gyro in roll axis (logical frame), bias-corrected
    bool  valid;           // false if input RawIMUData was invalid or not yet seeded
};

class StateEstimator {
public:
    // Call every IMU tick (at IMU_LOOP_HZ).
    // First call self-initialises from accel (no gyro integration on tick 0).
    // Returns invalid IMUState if raw.valid is false.
    IMUState update(const RawIMUData& raw);

    const IMUState& getState() const { return _state; }

    // Force re-initialisation from accel on the next update() call.
    // Use after a hard reset or if the robot was moved while powered off.
    void reset() { _initialized = false; }

private:
    IMUState _state{};
    bool     _initialized{false};

    // dt is derived from IMU_LOOP_HZ — recalculate here if loop rate changes.
    static constexpr float DT_S = 1.0f / static_cast<float>(IMU_LOOP_HZ);

    // ── Logical axis frame ────────────────────────────────────────────────────
    // Intermediate representation after mounting remapping.
    // All filter math operates on this struct, never on raw sensor axes.
    struct LogicalAxes {
        float a_fwd;   // m/s² — accel component along robot-forward axis
        float a_left;  // m/s² — accel component along robot-left axis
        float a_up;    // m/s² — accel component along robot-up axis
        float g_pitch; // rad/s — rotation rate about robot lateral (left) axis
        float g_roll;  // rad/s — rotation rate about robot forward axis
    };

    // Remap raw IMU axes to logical robot frame per IMU_MOUNTING setting.
    // This is the only function that knows about physical sensor orientation.
    static LogicalAxes remapAxes(const RawIMUData& d);

    // Accel-derived tilt — noisy but drift-free; weighted at (1 - CF_ALPHA).
    static float accelPitch(const LogicalAxes& l);
    static float accelRoll (const LogicalAxes& l);
};