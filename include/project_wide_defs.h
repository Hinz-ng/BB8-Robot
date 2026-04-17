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
constexpr uint32_t SPI_FREQ = 4000000; // 4 MHz — conservative for bringup

// ── Servo Pins (LEDC) ────────────────────────────────────────────────────────
// Drive servos — continuous rotation, one per side (differential drive)
constexpr int PIN_SERVO_LEFT   = 4;   // GPIO4 — left wheel
constexpr int PIN_SERVO_RIGHT  = 5;   // GPIO5 — right wheel
// Head servos (reserved for future phase)
constexpr int PIN_SERVO_HEAD_PAN  = 6;
constexpr int PIN_SERVO_HEAD_TILT = 7;

constexpr int LEDC_FREQ_HZ         = 50;   // Hz — standard servo PWM
constexpr int LEDC_RESOLUTION_BITS = 14;   // 14-bit: 0–16383
// At 50 Hz, 14-bit: 1 tick ≈ 1.22 µs  (period = 20 ms / 16384)
constexpr int SERVO_PULSE_MIN_US   = 1000; // µs — full speed one direction
constexpr int SERVO_PULSE_MID_US   = 1500; // µs — neutral / stop
constexpr int SERVO_PULSE_MAX_US   = 2000; // µs — full speed other direction

// ── Drive servo mounting flags ────────────────────────────────────────────────
// Set true for any wheel that spins the wrong direction during bringup.
// Verify: vel_y=+0.2 should move the sphere forward. If a wheel goes backward,
// set its REVERSED flag to true.  Do not change the mixing math in motion_controller.cpp.
constexpr bool DRIVE_LEFT_REVERSED  = false; // flip after bringup if needed
constexpr bool DRIVE_RIGHT_REVERSED = true;  // typical for mirrored mounting

// ── Drive speed ───────────────────────────────────────────────────────────────
constexpr float DRIVE_SPEED_DEFAULT = 0.5f;  // initial scale on boot (0.0–1.0)
constexpr float DRIVE_SPEED_MAX     = 1.0f;  // ceiling enforced by MotionController

// ── OE / E-stop ──────────────────────────────────────────────────────────────
constexpr int  PIN_OE              = 2;    // active LOW — gates drive servo power
constexpr uint32_t OE_BOOT_HOLD_MS = 1000; // hold outputs disabled at boot

// ── Complementary Filter ─────────────────────────────────────────────────────
// DEPRECATED — replaced by LSM6DSV16XTR SFLP on-chip fusion (Phase 4+).
// Retained for reference. Remove when SFLP is confirmed stable on hardware.
// constexpr float CF_ALPHA = 0.98f;

// ── Sphere Controller ────────────────────────────────────────────────────────
constexpr float SPHERE_KP_PITCH   = 0.0f; // tune after bringup
constexpr float SPHERE_KD_PITCH   = 0.0f;
constexpr float SPHERE_KP_ROLL    = 0.0f;
constexpr float SPHERE_KD_ROLL    = 0.0f;
constexpr float SPHERE_KP_MAX     = 50.0f;
constexpr float SPHERE_KD_MAX     = 10.0f;
constexpr float SPHERE_ESTOP_PITCH_DEG = 30.0f; // fall threshold

// ── IMU Mounting Orientation ──────────────────────────────────────────────────
// Defines how the physical IMU is oriented on the chassis.
// The state estimator uses this to remap raw axes to logical robot axes
// (forward, left, up) before running the complementary filter.
//
// At rest on a flat surface, verify by Serial-printing raw accel:
//   STANDARD:        az ≈ +9.81  (Z points up)
//   Z_FORWARD_Y_UP:  ay ≈ +9.81  (Y points up, Z points forward)
//   Z_FORWARD_X_UP:  ax ≈ +9.81  (X points up, Z points forward)
//
// Change only this line after physical mounting is confirmed.
enum class IMUMounting { STANDARD, Z_FORWARD_Y_UP, Z_FORWARD_X_UP };
constexpr IMUMounting IMU_MOUNTING = IMUMounting::Z_FORWARD_Y_UP;