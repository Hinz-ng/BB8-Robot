// module:  motion_controller.h
// layer:   3 — motion control; pure drive logic, no sensor reads
// purpose: differential drive mixing; translates (vel_x, vel_y) → servo PWM
// inputs:  vel_x, vel_y in [-1, +1]  (from WebComm DriveCallback)
//          speed_scale in [0, 1]     (from WebComm SpeedCallback)
// outputs: LEDC PWM on PIN_SERVO_LEFT, PIN_SERVO_RIGHT
// deps:    project_wide_defs.h, Arduino.h (LEDC — arduino-esp32 v3 API)
// date:    2025
//
// Differential drive mixing (vel_y = forward, vel_x = rightward turn):
//   left  = clamp( (vel_y + vel_x) * speed_scale )
//   right = clamp( (vel_y - vel_x) * speed_scale )
//
// MOUNTING CORRECTION:
//   DRIVE_LEFT_REVERSED / DRIVE_RIGHT_REVERSED in project_wide_defs.h flip
//   sign for physically mirrored servo mounting without touching the math.
//   First bringup step: set vel_y=+0.2, observe which direction each wheel
//   spins, and set the flag for any wheel that spins backward.
//
// OE PIN (future):
//   PIN_OE (GPIO2, active-low) is defined in project_wide_defs.h but not yet
//   managed here. Add oe_begin() / oe_estop() integration in a future phase
//   once the e-stop decision path is designed.

#pragma once
#include "project_wide_defs.h"

class MotionController {
public:
    // Call once in setup() after Serial is ready.
    // Attaches LEDC channels and writes neutral (SERVO_PULSE_MID_US) to both servos.
    // Returns false if LEDC attach fails — check pin conflicts.
    bool begin();

    // Set drive velocity. Called from WebComm DriveCallback each joystick update.
    //   vel_x: positive = rightward turn,  range [-1, +1]
    //   vel_y: positive = forward,          range [-1, +1]
    // Applies current speed_scale. Stores last command so setSpeedScale()
    // can re-apply immediately without waiting for the next joystick event.
    void setDrive(float vel_x, float vel_y);

    // Set speed scale multiplier. Called from WebComm SpeedCallback.
    // 0.0 = servos at neutral (stopped), 1.0 = full speed.
    // Clipped to [0.0, DRIVE_SPEED_MAX]. Re-applies last velocity immediately.
    void setSpeedScale(float scale);

    // Zero drive and write neutral pulse to both servos unconditionally.
    // Does NOT go through setDrive() — safe to call even if scale==0.
    void stop();

    float getSpeedScale() const { return _scale; }

private:
    float _scale{DRIVE_SPEED_DEFAULT};
    float _lastVelX{0.0f};   // stored so setSpeedScale() can re-apply instantly
    float _lastVelY{0.0f};

    // Map normalised value [-1, +1] → PWM microseconds [MIN_US, MAX_US].
    // Linear: us = MID + norm * (MAX - MID).  Symmetric range assumed (500 µs each side).
    // If SERVO_PULSE_MIN/MID/MAX_US change, this automatically adapts.
    static uint16_t normToUs(float norm);

    // Convert microseconds → LEDC duty count.
    // duty = us * (2^RESOLUTION - 1) / period_us.
    // Recalculate if LEDC_FREQ_HZ or LEDC_RESOLUTION_BITS change.
    static uint32_t usToDuty(uint16_t us);

    static float clamp1(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }
    static float clampRange(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
};