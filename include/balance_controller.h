// module:  balance_controller.h
// layer:   3 — motion control; pure computation, no hardware I/O
// purpose: PD balance controller for BB8 sphere robot.
//          Reads IMUState (pitch/roll + rates from logical gyro frame), outputs
//          velocity corrections that are mixed with user drive commands in main.cpp.
// inputs:  IMUState from state_estimator, user_vel_x/y from WebComm callback [-1,+1]
// outputs: BalanceOutput — velocity corrections, e-stop flag, telemetry fields
// deps:    state_estimator.h, project_wide_defs.h
// date:    2025
//
// Control law (normalized, dimensionless gains):
//   pitch_setpoint   = user_vel_y * SPHERE_BALANCE_LEAN_DEG_PER_UNIT   [feedforward]
//   pitch_error      = pitch_deg - pitch_setpoint
//   pitch_norm       = pitch_error   / SPHERE_ESTOP_PITCH_DEG           [-1, +1] at boundary
//   rate_norm        = pitch_rate_degs / SPHERE_BALANCE_RATE_REF_DEGS   [-1, +1] at ref rate
//   correction_vel_y = -SIGN * clamp(Kp*pitch_norm + Kd*rate_norm, ±CORRECTION_MAX)
//
// Normalized gains are dimensionless:
//   Kp = 0.5 → at 15° tilt (half of estop), apply 50% of CORRECTION_MAX.
//   Kd = 0.3 → at 180°/s, apply 30% of CORRECTION_MAX regardless of static angle.
//
// MANDATORY — Sign verification (Step 0 of tuning guide):
//   Enable balance with Kp=0.5, Kd=0.0. Tilt robot ~10° forward (nose down).
//   Expected: robot drives forward (positive vel_y correction) to catch the lean.
//   If it drives backward: set SPHERE_BALANCE_PITCH_SIGN = -1 in project_wide_defs.h
//   and reflash. This check MUST happen before any further gain tuning.
//
// E-stop:
//   Latches when |pitch| or |roll| > SPHERE_ESTOP_PITCH_DEG.
//   Auto-clears when BOTH return below 80% of threshold.
//   Independent of balance enable state — always active.
//   Manual clear: call resetEstop() after placing robot upright.

#pragma once
#include "state_estimator.h"
#include "project_wide_defs.h"

// ── BalanceConfig ─────────────────────────────────────────────────────────────
struct BalanceConfig {
    float kp_pitch = SPHERE_BALANCE_KP_PITCH_DEFAULT; // dimensionless — see control law above
    float kd_pitch = SPHERE_BALANCE_KD_PITCH_DEFAULT;
    float kp_roll  = SPHERE_BALANCE_KP_ROLL_DEFAULT;
    float kd_roll  = SPHERE_BALANCE_KD_ROLL_DEFAULT;
    bool  enabled  = false; // balance disabled on boot; enable via UI after sign is verified
};

// ── BalanceOutput ─────────────────────────────────────────────────────────────
struct BalanceOutput {
    float correction_vel_y  = 0.0f;  // vel [-1, +1] — add to user vel_y (pitch axis)
    float correction_vel_x  = 0.0f;  // vel [-1, +1] — add to user vel_x (roll axis)
    float pitch_error_deg   = 0.0f;  // deg — pitch - setpoint; for telemetry
    float pitch_rate_degs   = 0.0f;  // deg/s — from IMU logical frame; for telemetry
    float roll_error_deg    = 0.0f;  // deg — for telemetry
    bool  estop_active      = false; // true if |pitch| or |roll| > SPHERE_ESTOP_PITCH_DEG
    bool  balance_enabled   = false; // mirrors BalanceConfig.enabled; for UI sync
    bool  valid             = false; // false if IMUState input was invalid
};

// ── BalanceController ─────────────────────────────────────────────────────────
class BalanceController {
public:
    // Call once per control loop tick at IMU_LOOP_HZ (100 Hz).
    // user_vel_x, user_vel_y: raw drive commands from WebComm [-1, +1].
    // Returns corrections to mix with user commands in main.cpp before calling
    // motionController.setDrive(). When balance is disabled, corrections are 0
    // but e-stop detection still runs.
    BalanceOutput update(const IMUState& est, float user_vel_x, float user_vel_y);

    // Replace current config atomically (float writes are single-instruction on ESP32).
    void setConfig(const BalanceConfig& cfg) { _cfg = cfg; }
    const BalanceConfig& getConfig() const   { return _cfg; }

    // Last output — used by WebComm::broadcastState() for telemetry without re-running math.
    const BalanceOutput& getLastOutput() const { return _lastOutput; }

    // Clear e-stop latch manually. Normal operation: latch auto-clears when the
    // robot returns below 80% of SPHERE_ESTOP_PITCH_DEG on both axes.
    void resetEstop() { _estopLatched = false; }

private:
    BalanceConfig _cfg{};
    BalanceOutput _lastOutput{};
    bool          _estopLatched{false};

    static float clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};