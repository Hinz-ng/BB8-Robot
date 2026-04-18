// module:  balance_controller.cpp
// layer:   3 — pure computation; no hardware I/O
// purpose: PD pitch+roll balance for BB8 sphere robot
// date:    2025
//
// NOTE: Arduino.h intentionally excluded — this layer has no hardware I/O.
//       Arduino.h's RAD_TO_DEG macro is not used here; kRadToDeg is defined locally.

#include "balance_controller.h"
#include <cmath>  // fabsf — via <cmath>, not Arduino.h

// kRadToDeg: named to avoid collision with Arduino.h's RAD_TO_DEG macro.
static constexpr float kRadToDeg = 180.0f / 3.14159265f;

BalanceOutput BalanceController::update(const IMUState& est,
                                         float           user_vel_x,
                                         float           user_vel_y)
{
    BalanceOutput out;
    out.balance_enabled = _cfg.enabled;

    if (!est.valid) {
        // Propagate invalid IMU state — caller should not apply correction.
        _lastOutput = out;
        return out;  // out.valid = false
    }

    out.valid = true;

    const float pitch_deg      = est.pitch_deg;
    const float roll_deg       = est.roll_deg;
    // Convert logical-frame gyro rates (rad/s) to deg/s for the Kd term.
    // pitch_rate_rads is already bias-corrected and remapped by StateEstimator.
    // Using the direct gyro rate — NOT d(pitch_deg)/dt — avoids amplifying
    // filter output noise through numerical differentiation.
    const float pitch_rate_degs = est.pitch_rate_rads * kRadToDeg;
    const float roll_rate_degs  = est.roll_rate_rads  * kRadToDeg;

    // ── Telemetry snapshot ───────────────────────────────────────────────────
    // Populated unconditionally so the UI always shows current state.
    out.pitch_error_deg = pitch_deg;   // before setpoint subtraction; for raw display
    out.pitch_rate_degs = pitch_rate_degs;
    out.roll_error_deg  = roll_deg;

    // ── E-stop — always active regardless of balance enable state ────────────
    // Any axis exceeding the threshold latches the stop immediately.
    // Hysteresis: auto-clear requires BOTH axes below 80% of threshold.
    // This prevents rapid latch/clear cycling near the boundary.
    // Do NOT call motionController.stop() here — that call lives in main.cpp
    // to preserve layer separation.
    const float estopThresh = static_cast<float>(SPHERE_ESTOP_PITCH_DEG);
    if (fabsf(pitch_deg) > estopThresh || fabsf(roll_deg) > estopThresh) {
        _estopLatched = true;
    }
    if (_estopLatched
        && fabsf(pitch_deg) < estopThresh * 0.80f
        && fabsf(roll_deg)  < estopThresh * 0.80f) {
        _estopLatched = false;
    }
    out.estop_active = _estopLatched;

    // ── Skip PD output when disabled or e-stopped ────────────────────────────
    if (!_cfg.enabled || _estopLatched) {
        // Corrections remain zero from struct initialisation.
        _lastOutput = out;
        return out;
    }

    // ── Lean feedforward ──────────────────────────────────────────────────────
    // When SPHERE_BALANCE_LEAN_DEG_PER_UNIT > 0, the desired pitch setpoint
    // is proportional to the user's commanded velocity. This means the robot
    // leans into intentional motion rather than fighting it.
    //
    // Leave at 0 until basic PD is confirmed stable on hardware.
    // Typical useful range after tuning: 3–8 degrees at full throttle.
    //   At 5°: full-speed forward (vel_y=1.0) → setpoint = +5° forward lean.
    const float pitch_setpoint_deg = user_vel_y
                                   * static_cast<float>(SPHERE_BALANCE_LEAN_DEG_PER_UNIT);

    // ── Axis errors ───────────────────────────────────────────────────────────
    float pitch_err = pitch_deg - pitch_setpoint_deg;
    float roll_err  = roll_deg;  // roll setpoint is always 0 (symmetric chassis)

    // Deadband — prevents servo jitter when nearly upright.
    // If servos chatter at rest, increase SPHERE_BALANCE_DEADBAND_DEG.
    // If near-zero corrections feel sluggish, decrease it.
    if (fabsf(pitch_err) < static_cast<float>(SPHERE_BALANCE_DEADBAND_DEG)) pitch_err = 0.0f;
    if (fabsf(roll_err)  < static_cast<float>(SPHERE_BALANCE_DEADBAND_DEG)) roll_err  = 0.0f;

    // ── Normalize inputs ──────────────────────────────────────────────────────
    // Normalization makes Kp and Kd dimensionless and their values comparable:
    //   pitch_norm = 1.0 when at the estop boundary (30°)
    //   rate_norm  = 1.0 at SPHERE_BALANCE_RATE_REF_DEGS (180°/s)
    //
    // Why normalize instead of raw degrees?
    //   Raw degree gains are IMU-range-dependent and hard to transfer between
    //   tuning sessions. Normalized gains read as "fraction of max correction
    //   at a given reference condition" — directly interpretable.
    const float pitch_norm = pitch_err        / estopThresh;
    const float rate_norm  = pitch_rate_degs  / static_cast<float>(SPHERE_BALANCE_RATE_REF_DEGS);
    const float roll_norm  = roll_err         / estopThresh;
    const float rrate_norm = roll_rate_degs   / static_cast<float>(SPHERE_BALANCE_RATE_REF_DEGS);

    // ── PD control ────────────────────────────────────────────────────────────
    // Sign: -PITCH_SIGN * (Kp * pitch_norm + Kd * rate_norm)
    //
    // With SPHERE_BALANCE_PITCH_SIGN = +1:
    //   Positive pitch error (nose up) → negative correction (backward drive).
    //   Physically: nose-up means sphere overshot forward; drive backward to reduce
    //   the lag between sphere and drive unit.
    //
    // If enabling balance INCREASES oscillation, flip SPHERE_BALANCE_PITCH_SIGN
    // to -1 in project_wide_defs.h without changing any gain values.
    //
    // The Kd term is the primary oscillation damper.
    // At 180°/s pitch rate with Kd=0.3: correction = 0.3 * 0.7 = 0.21 vel units.
    // This braking force opposes the swing velocity before angle grows large.
    const float raw_y = -static_cast<float>(SPHERE_BALANCE_PITCH_SIGN)
                        * (_cfg.kp_pitch * pitch_norm + _cfg.kd_pitch * rate_norm);

    const float raw_x = -static_cast<float>(SPHERE_BALANCE_ROLL_SIGN)
                        * (_cfg.kp_roll  * roll_norm  + _cfg.kd_roll  * rrate_norm);

    // ── Clamp output ──────────────────────────────────────────────────────────
    // Prevents balance correction from overwhelming user drive input entirely.
    // At CORRECTION_MAX = 0.70, a user drive of vel_y = 1.0 plus max correction
    // saturates setDrive at 1.0 (clamped by MotionController internally).
    // Reduce CORRECTION_MAX if balance feels too aggressive during manual control.
    const float maxCorr = static_cast<float>(SPHERE_BALANCE_CORRECTION_MAX);
    out.correction_vel_y = clamp(raw_y, -maxCorr, maxCorr);
    out.correction_vel_x = clamp(raw_x, -maxCorr, maxCorr);

    _lastOutput = out;
    return out;
}