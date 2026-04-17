// module:  state_estimator.cpp
// layer:   2 — sensor fusion; pure computation, no hardware I/O
// purpose: gravity vector extraction from SFLP quaternion → pitch + roll
// date:    2025
//
// NOTE: Arduino.h intentionally excluded — this layer has no hardware I/O.
//       Arduino.h defines RAD_TO_DEG as a macro; use kRadToDeg below.

#include "state_estimator.h"
#include <cmath>   // atan2f, sqrtf

// ── update ────────────────────────────────────────────────────────────────────
IMUState StateEstimator::update(const SFLPData& sflp) {
    if (!sflp.valid) {
        _state.valid = false;
        return _state;
    }

    // ── Gravity vector extraction ─────────────────────────────────────────────
    // Extract the "world up" direction in the current chip frame.
    // This equals Rᵀ * world_up_at_boot, where world_up_at_boot is the chip
    // axis that was aligned with gravity when SFLP initialised.
    //
    // For unit quaternion q=(qw,qx,qy,qz) representing body-to-world rotation,
    // the rotation matrix R has rows:
    //   R₁ = [1−2(qy²+qz²),  2(qxqy−qwqz),  2(qxqz+qwqy)]
    //   R₂ = [2(qxqy+qwqz),  1−2(qx²+qz²),  2(qyqz−qwqx)]
    //   R₃ = [2(qxqz−qwqy),  2(qyqz+qwqx),  1−2(qx²+qy²)]
    //
    // g_chip = Rᵀ * world_up_at_boot
    //   STANDARD       (Z=up):  world_up=[0,0,1] → g = R₃
    //   Z_FORWARD_Y_UP (Y=up):  world_up=[0,1,0] → g = R₂
    //   Z_FORWARD_X_UP (X=up):  world_up=[1,0,0] → g = R₁
    //
    // Pitch/roll signs verified algebraically against small-angle test
    // quaternions (rotation about each axis by angle θ, confirmed θ recovered).
    // ─────────────────────────────────────────────────────────────────────────

    const float qw = sflp.qw, qx = sflp.qx, qy = sflp.qy, qz = sflp.qz;
    float gx, gy, gz;

    switch (IMU_MOUNTING) {

    case IMUMounting::STANDARD:
        // Chip Z = up at boot → g = R₃
        gx = 2.0f*(qx*qz - qw*qy);
        gy = 2.0f*(qy*qz + qw*qx);
        gz = 1.0f - 2.0f*(qx*qx + qy*qy);
        // pitch: forward=X, up=Z → atan2(-gx, gz)
        // roll:  left=Y,    up=Z → atan2( gy, gz)
        _state.pitch_rad = atan2f(-gx, gz);
        _state.roll_rad  = atan2f( gy, gz);
        break;

    case IMUMounting::Z_FORWARD_Y_UP:
        // Chip Y = up at boot → g = R₂
        gx = 2.0f*(qx*qy + qw*qz);
        gy = 1.0f - 2.0f*(qx*qx + qz*qz);
        gz = 2.0f*(qy*qz - qw*qx);
        // pitch: forward=Z, up=Y → atan2(-gz, gy)
        // roll:  left=X,   up=Y → atan2( gx, gy)
        _state.pitch_rad = atan2f(-gz, gy);
        _state.roll_rad  = atan2f( gx, gy);
        break;

    case IMUMounting::Z_FORWARD_X_UP:
        // Chip X = up at boot → g = R₁
        gx = 1.0f - 2.0f*(qy*qy + qz*qz);
        gy = 2.0f*(qx*qy - qw*qz);
        gz = 2.0f*(qx*qz + qw*qy);
        // pitch: forward=Z, up=X → atan2(-gz, gx)
        // roll:  right=Y (so left=−Y), up=X → atan2(-gy, gx)
        _state.pitch_rad = atan2f(-gz, gx);
        _state.roll_rad  = atan2f(-gy, gx);
        break;
    }

   // Guard NaN — degenerate quaternion inputs can produce NaN via atan2f(0,0).
    // Without this, a single bad SFLP packet corrupts the broadcast JSON and
    // kills the entire WebSocket display until the next valid packet.
    if (std::isnan(_state.pitch_rad) || std::isnan(_state.roll_rad)) {
        _state.pitch_rad = 0.0f;
        _state.roll_rad  = 0.0f;
        _state.pitch_deg = 0.0f;
        _state.roll_deg  = 0.0f;
        _state.valid     = false;
        return _state;
    }

    constexpr float kRadToDeg = 180.0f / 3.14159265f;
    _state.pitch_deg = _state.pitch_rad * kRadToDeg;
    _state.roll_deg  = _state.roll_rad  * kRadToDeg;
    _state.valid     = true;
    return _state;
}