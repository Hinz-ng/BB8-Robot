// module:  state_estimator.h
// layer:   2 — sensor fusion; no hardware I/O
// purpose: converts SFLP quaternion to pitch + roll angle estimates via
//          gravity vector projection — mounting-agnostic, no gimbal lock
// inputs:  SFLPData (from imu.h) — on-chip game rotation vector
// outputs: IMUState struct (pitch_rad, roll_rad, pitch_deg, roll_deg, valid)
// deps:    imu.h, project_wide_defs.h
// date:    2025
//
// Algorithm:
//   SFLP initialises to q=[1,0,0,0] at boot (stationary required).
//   The quaternion tracks orientation changes from that boot pose.
//   The "world up" direction in chip frame = Rᵀ * boot_up_axis, where
//   boot_up_axis is the chip axis aligned with gravity at boot (= IMU_MOUNTING).
//   Pitch and roll are atan2 projections of that gravity vector.
//
//   This replaces the complementary filter entirely — no alpha, no DT_S,
//   no gyro integration in software. All sensor fusion runs on the chip DSP.
//
// Axis sign convention (VERIFY on hardware before enabling PD controller):
//   pitch > 0 → nose up
//   roll  > 0 → right side down
//   If signs are inverted, correct in project_wide_defs.h or here — do not
//   introduce correction factors elsewhere.

#pragma once
#include "imu.h"
#include "project_wide_defs.h"
#include <cmath>

struct IMUState {
    float pitch_rad;  // rad — positive = nose up
    float roll_rad;   // rad — positive = right side down
    float pitch_deg;  // deg — convenience copy for telemetry + PD inputs
    float roll_deg;   // deg
    bool  valid;      // false if SFLPData was invalid
};

class StateEstimator {
public:
    // Call every loop tick at IMU_LOOP_HZ.
    // Returns invalid IMUState if sflp.valid is false.
    IMUState update(const SFLPData& sflp);

    const IMUState& getState() const { return _state; }

    // reset() is retained for API compatibility but is a no-op with SFLP.
    // SFLP re-locks to gravity automatically within milliseconds of the
    // chip being held still — no software re-initialisation needed.
    void reset() {}

private:
    IMUState _state{};
};