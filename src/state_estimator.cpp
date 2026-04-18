// module:  state_estimator.cpp
// layer:   2 — sensor fusion; pure computation, no hardware I/O
// purpose: complementary filter for pitch + roll; mounting-agnostic via remapAxes()
// date:    2025
//
// NOTE: Arduino.h is intentionally excluded — this layer has no hardware I/O.
//       Arduino.h defines RAD_TO_DEG as a macro; use kRadToDeg below instead.

#include "state_estimator.h"
#include <cmath>  // atan2f, sqrtf — explicit inclusion, not via Arduino.h

// ── update ────────────────────────────────────────────────────────────────────
IMUState StateEstimator::update(const RawIMUData& raw) {
    if (!raw.valid) {
        _state.valid = false;
        return _state;
    }

    const LogicalAxes l = remapAxes(raw);
    const float ap = accelPitch(l);
    const float ar = accelRoll(l);

    if (!_initialized) {
        // Seed from accel on first call — prevents the large convergence
        // transient you would otherwise see for the first ~100 ticks.
        _state.pitch_rad = ap;
        _state.roll_rad  = ar;
        _initialized     = true;
    } else {
        // Complementary filter — mounting-agnostic because all axis
        // dependency is resolved by remapAxes() above.
        //
        // CF_ALPHA ≈ 0.98 → gyro dominates short transients;
        // accel corrects drift over ~0.5 s time constant.
        // Recalculate if IMU_LOOP_HZ changes:
        //   alpha ≈ 1 - (1 / (tau_sec * IMU_LOOP_HZ))   [tau = 0.5 s]
        _state.pitch_rad = CF_ALPHA * (_state.pitch_rad + l.g_pitch * DT_S)
                         + (1.0f - CF_ALPHA) * ap;

        _state.roll_rad  = CF_ALPHA * (_state.roll_rad  + l.g_roll  * DT_S)
                         + (1.0f - CF_ALPHA) * ar;
    }

    // kRadToDeg: named to avoid collision with Arduino.h's RAD_TO_DEG macro.
    constexpr float kRadToDeg = 180.0f / 3.14159265f;
    _state.pitch_deg = _state.pitch_rad * kRadToDeg;
    _state.roll_deg  = _state.roll_rad  * kRadToDeg;
    _state.valid     = true;

    return _state;
}

// ── remapAxes ─────────────────────────────────────────────────────────────────
// Transforms raw sensor axes into logical robot frame (forward/left/up).
// This is the ONLY function that knows about physical sensor orientation.
// All other filter logic is mounting-agnostic.
//
// Mounting constants (project_wide_defs.h):
//   STANDARD:        Z_imu=up,  X_imu=forward → at rest az≈+9.81
//   Z_FORWARD_Y_UP:  Z_imu=fwd, Y_imu=up      → at rest ay≈+9.81
//   Z_FORWARD_X_UP:  Z_imu=fwd, X_imu=up      → at rest ax≈+9.81
//
// Derivation (right-hand frame, F=forward, L=left, U=up):
//   Z_FORWARD_Y_UP: X_imu = F×L/U = robot_left  → a_left=+ax, g_pitch=+gx
//   Z_FORWARD_X_UP: Y_imu = F×U   = robot_right → a_left=-ay, g_pitch=-gy
//
// Verify on hardware: hold robot still, confirm which axis reads +9.81 in
// the Serial print, then set IMU_MOUNTING accordingly in project_wide_defs.h.
StateEstimator::LogicalAxes StateEstimator::remapAxes(const RawIMUData& d) {
    LogicalAxes l{};

    switch (IMU_MOUNTING) {

    case IMUMounting::STANDARD:
        // Default: Z up, X forward, Y left.
        // Pitch = rotation about Y (lateral). Roll = rotation about X (forward).
        l.a_fwd   =  d.accel_x_ms2;
        l.a_left  =  d.accel_y_ms2;
        l.a_up    =  d.accel_z_ms2;
        l.g_pitch =  d.gyro_y_rads;
        l.g_roll  =  d.gyro_x_rads;
        break;

    case IMUMounting::Z_FORWARD_Y_UP:
        // Z toward robot nose, Y up, X left.
        // Pitch = rotation about X (lateral=left). Roll = rotation about Z (forward).
        l.a_fwd   =  d.accel_z_ms2;
        l.a_left  =  d.accel_x_ms2;
        l.a_up    =  d.accel_y_ms2;
        l.g_pitch =  d.gyro_x_rads;
        l.g_roll  =  d.gyro_z_rads;
        break;

    case IMUMounting::Z_FORWARD_X_UP:
        // Z toward robot nose, X up, Y right (so robot-left = -Y).
        // Pitch = rotation about robot-left = -IMU_Y → negate gy.
        // Roll  = rotation about forward = IMU_Z → gz.
        l.a_fwd   =  d.accel_z_ms2;
        l.a_left  = -d.accel_y_ms2;   // Y_imu = robot-right; left = -right
        l.a_up    =  d.accel_x_ms2;
        l.g_pitch = -d.gyro_y_rads;   // positive pitch (nose up) = negative IMU-Y rotation
        l.g_roll  =  d.gyro_z_rads;
        break;
    }

    return l;
}

// ── accelPitch ────────────────────────────────────────────────────────────────
// Tilt about the lateral axis from accel alone.
// Formula: atan2(-a_fwd, sqrt(a_left² + a_up²))
// Returns zero when a_fwd=0, a_up>0 (upright). Positive = nose up.
// Works for all mountings because remapAxes() has already normalised the axes.
float StateEstimator::accelPitch(const LogicalAxes& l) {
    const float denom = sqrtf(l.a_left * l.a_left + l.a_up * l.a_up);
    return atan2f(-l.a_fwd, denom);
}

// ── accelRoll ─────────────────────────────────────────────────────────────────
// Tilt about the forward axis from accel alone.
// Formula: atan2(a_left, a_up)
// Returns zero when a_left=0, a_up>0 (upright). Positive = right side down.
float StateEstimator::accelRoll(const LogicalAxes& l) {
    return atan2f(l.a_left, l.a_up);
}