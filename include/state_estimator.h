// module:  state_estimator.h
// layer:   2 — sensor fusion; no hardware I/O
// purpose: converts SFLP quaternion to pitch + roll angle estimates via
//          gravity vector projection — mounting-agnostic, no gimbal lock
// inputs:  SFLPData (from imu.h) — on-chip game rotation vector
// outputs: IMUState struct (pitch_rad, roll_rad, pitch_deg, roll_deg, valid)
// deps:    imu.h, project_wide_defs.h
// date:    2025

#pragma once
#include "imu.h"
#include "project_wide_defs.h"
#include <cmath>

struct IMUState {
    float pitch_rad;
    float roll_rad;
    float pitch_deg;
    float roll_deg;
    bool  valid;
};

class StateEstimator {
public:
    // Call every loop tick. Returns invalid IMUState if sflp.valid is false.
    IMUState update(const SFLPData& sflp);

    const IMUState& getState() const { return _state; }

    // No-op with SFLP — chip re-locks to gravity automatically when held still.
    void reset() {}

private:
    IMUState _state{};
};