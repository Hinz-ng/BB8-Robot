// module:  state_estimator.cpp
// layer:   2 — sensor fusion; pure computation, no hardware I/O
// purpose: gravity vector extraction from SFLP quaternion → pitch + roll
// date:    2025
//
// NOTE: Arduino.h intentionally excluded — this layer has no hardware I/O.
//       Arduino.h defines RAD_TO_DEG as a macro; use kRadToDeg below.

#include "state_estimator.h"
#include <cmath>

IMUState StateEstimator::update(const SFLPData& sflp) {
    if (!sflp.valid) {
        _state.valid = false;
        return _state;
    }

    const float qw = sflp.qw, qx = sflp.qx, qy = sflp.qy, qz = sflp.qz;
    float gx, gy, gz;

    // Gravity vector = Rᵀ * boot_up_axis, where boot_up_axis is whichever
    // chip axis was aligned with gravity when SFLP initialised.
    // Select via IMU_MOUNTING in project_wide_defs.h.
    switch (IMU_MOUNTING) {

    case IMUMounting::STANDARD:
        // Chip Z = up at boot → g = R₃
        gx =  2.0f*(qx*qz - qw*qy);
        gy =  2.0f*(qy*qz + qw*qx);
        gz =  1.0f - 2.0f*(qx*qx + qy*qy);
        _state.pitch_rad = atan2f(-gx, gz);
        _state.roll_rad  = atan2f( gy, gz);
        break;

    case IMUMounting::Z_FORWARD_Y_UP:
        // Chip Y = up at boot → g = R₂
        gx =  2.0f*(qx*qy + qw*qz);
        gy =  1.0f - 2.0f*(qx*qx + qz*qz);
        gz =  2.0f*(qy*qz - qw*qx);
        _state.pitch_rad = atan2f(-gz, gy);
        _state.roll_rad  = atan2f( gx, gy);
        break;

    case IMUMounting::Z_FORWARD_X_UP:
        // Chip X = up at boot → g = R₁
        gx =  1.0f - 2.0f*(qy*qy + qz*qz);
        gy =  2.0f*(qx*qy - qw*qz);
        gz =  2.0f*(qx*qz + qw*qy);
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