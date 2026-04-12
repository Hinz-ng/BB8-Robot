// module:  state_estimator.cpp
// layer:   2 — sensor fusion; pure computation, no hardware I/O
// purpose: complementary filter for pitch + roll angle estimates
// date:    2025

#include "state_estimator.h"
#include <cmath>  // atan2f, sqrtf — explicit, not via Arduino.h

// NOTE: Arduino.h is intentionally excluded. Layer 2 has no hardware I/O.
// It also defines RAD_TO_DEG as a macro which collides with any local constant
// of the same name. Use kRadToDeg below instead.

// ── update ────────────────────────────────────────────────────────────────────
IMUState StateEstimator::update(const RawIMUData& raw) {
    if (!raw.valid) {
        _state.valid = false;
        return _state;
    }

    const float ap = accelPitch(raw);
    const float ar = accelRoll(raw);

    if (!_initialized) {
        // Seed from accel on the very first call — no gyro integration yet.
        // Prevents the large transient you would otherwise see while the
        // filter converges from zero toward the true angle.
        _state.pitch_rad = ap;
        _state.roll_rad  = ar;
        _initialized     = true;
    } else {
        // Complementary filter:
        //   Gyro term:  fast, low-noise, but drifts over time
        //   Accel term: drift-free, but noisy and sensitive to linear accel
        //
        //   CF_ALPHA ≈ 0.98 (from project_wide_defs.h) → gyro dominates for
        //   short transients; accel corrects drift over ~0.5 s time constant.
        //
        //   Recalculate CF_ALPHA if IMU_LOOP_HZ changes:
        //     alpha ≈ 1 - (1 / (tau_sec * IMU_LOOP_HZ))   [tau = 0.5 s]
        _state.pitch_rad = CF_ALPHA * (_state.pitch_rad + raw.gyro_y_rads * DT_S)
                         + (1.0f - CF_ALPHA) * ap;

        _state.roll_rad  = CF_ALPHA * (_state.roll_rad  + raw.gyro_x_rads * DT_S)
                         + (1.0f - CF_ALPHA) * ar;
    }

    // Degree copies for telemetry and PD controller readability.
    // kRadToDeg is named to avoid collision with Arduino.h's RAD_TO_DEG macro.
    constexpr float kRadToDeg = 180.0f / 3.14159265f;
    _state.pitch_deg = _state.pitch_rad * kRadToDeg;
    _state.roll_deg  = _state.roll_rad  * kRadToDeg;
    _state.valid     = true;

    return _state;
}

// ── accelPitch ────────────────────────────────────────────────────────────────
// Tilt about Y axis from accel alone: atan2(-ax, sqrt(ay² + az²))
// Zero when ax=0, az=+g (upright, Z pointing up). Positive = nose up.
// AXIS DEPENDENCY: ax=forward, az=up at rest. Verify Serial print before use.
float StateEstimator::accelPitch(const RawIMUData& d) {
    const float denom = sqrtf(d.accel_y_ms2 * d.accel_y_ms2
                             + d.accel_z_ms2 * d.accel_z_ms2);
    return atan2f(-d.accel_x_ms2, denom);
}

// ── accelRoll ─────────────────────────────────────────────────────────────────
// Tilt about X axis from accel alone: atan2(ay, az)
// Zero when ay=0, az=+g (upright). Positive = right side down.
// AXIS DEPENDENCY: ay=left, az=up at rest. Verify Serial print before use.
float StateEstimator::accelRoll(const RawIMUData& d) {
    return atan2f(d.accel_y_ms2, d.accel_z_ms2);
}