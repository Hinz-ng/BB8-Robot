// module:  project_wide_defs.h
// layer:   global constants — single source of truth
// purpose: all tunable values, pin assignments, loop rates, safety limits
// date:    2025

#pragma once
#include <cstdint>

// ── Loop Rates ──────────────────────────────────────────────────────────────
constexpr uint32_t IMU_LOOP_HZ          = 100;   // Hz; must match CTRL1/CTRL2 ODR
constexpr uint32_t CONTROL_LOOP_HZ      = 100;   // Hz; sphere controller rate
constexpr uint32_t WEBCOMM_BROADCAST_HZ = 20;    // Hz; telemetry broadcast

// ── SPI Pins (ESP32-S3 Super Mini) ──────────────────────────────────────────
constexpr int PIN_SPI_SCLK  = 12;
constexpr int PIN_SPI_MOSI  = 11;
constexpr int PIN_SPI_MISO  = 13;
constexpr int PIN_IMU_CS    = 10;
constexpr uint32_t SPI_FREQ = 4'000'000; // 4 MHz — conservative for bringup

// ── Servo Pins (LEDC channels) ───────────────────────────────────────────────
// Drive servos (continuous rotation, omniwheel contact)
constexpr int PIN_SERVO_DRIVE_X  = 4;  // forward/back axis
constexpr int PIN_SERVO_DRIVE_Y  = 5;  // lateral axis
// Head servos
constexpr int PIN_SERVO_HEAD_PAN  = 6;
constexpr int PIN_SERVO_HEAD_TILT = 7;

constexpr int LEDC_FREQ_HZ        = 50;    // standard servo PWM
constexpr int LEDC_RESOLUTION_BITS = 14;   // 0–16383 range
// At 50Hz, 14-bit: 1 tick ≈ 1.22 µs  (period = 20ms / 16384)
constexpr int SERVO_PULSE_MIN_US  = 1000;  // µs → 0°
constexpr int SERVO_PULSE_MID_US  = 1500;  // µs → 90° / stop for continuous
constexpr int SERVO_PULSE_MAX_US  = 2000;  // µs → 180°

// ── OE / E-stop ──────────────────────────────────────────────────────────────
constexpr int  PIN_OE              = 2;    // active LOW — gates drive servo power
constexpr uint32_t OE_BOOT_HOLD_MS = 1000; // hold outputs disabled at boot

// ── Complementary Filter ─────────────────────────────────────────────────────
// Alpha ≈ 1 - 1/(tau_sec * IMU_LOOP_HZ). tau=0.5s, 100Hz → alpha=0.98
// Recalculate if IMU_LOOP_HZ changes.
constexpr float CF_ALPHA = 0.98f;

// ── Sphere Controller ────────────────────────────────────────────────────────
constexpr float SPHERE_KP_PITCH   = 0.0f; // tune after bringup
constexpr float SPHERE_KD_PITCH   = 0.0f;
constexpr float SPHERE_KP_ROLL    = 0.0f;
constexpr float SPHERE_KD_ROLL    = 0.0f;
constexpr float SPHERE_KP_MAX     = 50.0f;
constexpr float SPHERE_KD_MAX     = 10.0f;
constexpr float SPHERE_ESTOP_PITCH_DEG = 30.0f; // fall threshold