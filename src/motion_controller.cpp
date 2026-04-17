// module:  motion_controller.cpp
// layer:   3 — motion control; pure drive logic, no sensor reads
// purpose: differential drive mixing + LEDC servo output
// date:    2025
//
// LEDC compatibility:
//   arduino-esp32 v3: ledcAttach(pin, ...) + ledcWrite(pin, duty)
//   arduino-esp32 v2: ledcSetup(ch, ...) + ledcAttachPin(pin, ch) + ledcWrite(ch, duty)
// This file auto-selects the API by ESP_ARDUINO_VERSION_MAJOR.

#include "motion_controller.h"
#include "project_wide_defs.h"
#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

namespace {
constexpr uint8_t LEDC_CH_LEFT  = 0;
constexpr uint8_t LEDC_CH_RIGHT = 1;

bool attachServoPwm(uint8_t pin, uint8_t channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    return ledcAttach(pin, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
#else
    const double actual_freq = ledcSetup(channel, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
    if (actual_freq <= 0.0) return false;
    ledcAttachPin(pin, channel);
    return true;
#endif
}

void writeServoDuty(uint8_t pin, uint8_t channel, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
}
} // namespace

bool MotionController::begin() {
    bool ok = true;
    ok &= attachServoPwm(PIN_SERVO_LEFT,  LEDC_CH_LEFT);
    ok &= attachServoPwm(PIN_SERVO_RIGHT, LEDC_CH_RIGHT);

    if (!ok) {
        Serial.println("[MOTION] LEDC attach failed — check GPIO assignments for conflicts");
        return false;
    }

    // Write neutral immediately so servos don't twitch on power-up.
    // This must happen before any velocity command arrives.
    stop();

    Serial.printf("[MOTION] Ready. Left=GPIO%d  Right=GPIO%d  Scale=%.2f\n",
                  PIN_SERVO_LEFT, PIN_SERVO_RIGHT, _scale);
    return true;
}

void MotionController::setDrive(float vel_x, float vel_y) {
    _lastVelX = clamp1(vel_x);
    _lastVelY = clamp1(vel_y);

    // Differential mixing.
    // Positive vel_y = forward: both wheels spin forward at equal rate.
    // Positive vel_x = turn right: left wheel faster, right wheel slower.
    float left  = clamp1((_lastVelY + _lastVelX) * _scale);
    float right = clamp1((_lastVelY - _lastVelX) * _scale);

    // Correct for physically mirrored mounting without touching the sign convention.
    // Set DRIVE_LEFT_REVERSED = true in project_wide_defs.h if the left wheel spins
    // backward when vel_y is positive (forward command).  Same for right.
    if (DRIVE_LEFT_REVERSED)  left  = -left;
    if (DRIVE_RIGHT_REVERSED) right = -right;

    writeServoDuty(PIN_SERVO_LEFT,  LEDC_CH_LEFT,  usToDuty(normToUs(left)));
    writeServoDuty(PIN_SERVO_RIGHT, LEDC_CH_RIGHT, usToDuty(normToUs(right)));
}

void MotionController::setSpeedScale(float scale) {
    _scale = clampRange(scale, 0.0f, DRIVE_SPEED_MAX);

    // Re-apply last velocity immediately.
    // Without this, a speed slider change has no effect until the next
    // joystick event — which feels broken during a live speed adjustment.
    setDrive(_lastVelX, _lastVelY);

    Serial.printf("[MOTION] Speed scale → %.2f\n", _scale);
}

void MotionController::stop() {
    _lastVelX = 0.0f;
    _lastVelY = 0.0f;
    // Write neutral directly — not through setDrive() — so this path is safe
    // even if _scale is zero (setDrive with scale=0 would also write neutral,
    // but being explicit here removes any ambiguity in e-stop scenarios).
    const uint32_t neutral = usToDuty(SERVO_PULSE_MID_US);
    writeServoDuty(PIN_SERVO_LEFT,  LEDC_CH_LEFT,  neutral);
    writeServoDuty(PIN_SERVO_RIGHT, LEDC_CH_RIGHT, neutral);
}

// ── Private helpers ───────────────────────────────────────────────────────────

uint16_t MotionController::normToUs(float norm) {
    // norm [-1, +1] → microseconds [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US]
    // Piecewise mapping around midpoint:
    //   norm >= 0: MID_US + norm * (MAX_US - MID_US)
    //   norm <  0: MID_US + norm * (MID_US - MIN_US)
    // This keeps full-range behaviour correct even if neutral is not centered.
    const float n = clamp1(norm);
    const float mid = static_cast<float>(SERVO_PULSE_MID_US);
    const float posSpan = static_cast<float>(SERVO_PULSE_MAX_US - SERVO_PULSE_MID_US);
    const float negSpan = static_cast<float>(SERVO_PULSE_MID_US - SERVO_PULSE_MIN_US);
    const float span = (n >= 0.0f) ? posSpan : negSpan;
    int32_t us = static_cast<int32_t>(mid + n * span);
    if (us < SERVO_PULSE_MIN_US) us = SERVO_PULSE_MIN_US;
    if (us > SERVO_PULSE_MAX_US) us = SERVO_PULSE_MAX_US;
    return static_cast<uint16_t>(us);
}

uint32_t MotionController::usToDuty(uint16_t us) {
    // duty = us * (2^RESOLUTION - 1) / period_us
    // period_us = 1,000,000 / LEDC_FREQ_HZ  (20,000 µs at 50 Hz)
    // max_duty  = (1 << LEDC_RESOLUTION_BITS) - 1  (16383 at 14-bit)
    //
    // Integer math: us * max_duty / period_us.
    // At 14-bit / 50 Hz:  1500 µs → duty = 1500 * 16383 / 20000 = 12287.
    //
    // Recalculate comment if LEDC_FREQ_HZ or LEDC_RESOLUTION_BITS change.
    constexpr uint32_t PERIOD_US = 1000000UL / LEDC_FREQ_HZ;
    constexpr uint32_t MAX_DUTY  = (1U << LEDC_RESOLUTION_BITS) - 1U;
    return static_cast<uint32_t>(us) * MAX_DUTY / PERIOD_US;
}
 
