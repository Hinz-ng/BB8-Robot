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
// set its REVERSED flag to true. Do not change the mixing math in motion_controller.cpp.
constexpr bool DRIVE_LEFT_REVERSED  = false;
constexpr bool DRIVE_RIGHT_REVERSED = true;  // typical for mirrored mounting

// ── Drive speed ───────────────────────────────────────────────────────────────
constexpr float DRIVE_SPEED_DEFAULT = 1.0f;  // initial scale on boot (0.0–1.0)
constexpr float DRIVE_SPEED_MAX     = 1.0f;  // ceiling enforced by MotionController

// ── OE / E-stop ──────────────────────────────────────────────────────────────
constexpr int  PIN_OE              = 2;    // active LOW — gates drive servo power
constexpr uint32_t OE_BOOT_HOLD_MS = 1000; // hold outputs disabled at boot

// ── Complementary Filter ─────────────────────────────────────────────────────
// Note: LSM6DSV16XTR has on-chip SFLP fusion (Phase 4+ candidate).
// Complementary filter is the active approach until SFLP is validated on hardware.
constexpr float CF_ALPHA = 0.98f;
// Rate-dependent. Recalculate if IMU_LOOP_HZ changes:
//   alpha ≈ 1 - (1 / (tau_sec * IMU_LOOP_HZ))  [tau ≈ 0.5 s at alpha=0.98, 100 Hz]

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

// ── Sphere Safety ─────────────────────────────────────────────────────────────
constexpr float SPHERE_ESTOP_PITCH_DEG = 80.0f; // deg — hard stop above this tilt

// ── Balance Controller ────────────────────────────────────────────────────────
// Normalized gains: Kp=1.0 applies full CORRECTION_MAX at the estop boundary (30°).
//                   Kd=1.0 applies full CORRECTION_MAX at RATE_REF_DEGS (180 °/s).
// This makes gains dimensionless and directly interpretable:
//   Kp = 0.5 → at 15° tilt (half of estop), apply 50% of CORRECTION_MAX.
//   Kd = 0.3 → at 180°/s angular rate, apply 30% of CORRECTION_MAX.
//
// Bringup sequence (see tuning guide in response):
//   Step 0 — Verify SPHERE_BALANCE_PITCH_SIGN BEFORE enabling any gain.
//   Step 1 — Kp = 0.5, Kd = 0.0 → confirm restoring behaviour.
//   Step 2 — Increase Kp until onset of oscillation; back off 30%.
//   Step 3 — Add Kd to damp residual oscillation.
//   Step 4 — Enable lean feedforward after basic PD is stable.
//
// All gains tunable live via CMD:BALANCE_TUNE WebSocket command.
constexpr float SPHERE_BALANCE_KP_PITCH_DEFAULT  = 0.5f;  // start here; tune upward
constexpr float SPHERE_BALANCE_KD_PITCH_DEFAULT  = 0.0f;  // add AFTER Kp is verified
constexpr float SPHERE_BALANCE_KP_ROLL_DEFAULT   = 0.3f;
constexpr float SPHERE_BALANCE_KD_ROLL_DEFAULT   = 0.0f;

// WebSocket gain ceilings — clamped at parse time in webcomm.cpp.
// Raise only after servo lag is measured and phase margin is confirmed.
constexpr float SPHERE_BALANCE_KP_MAX            = 3.0f;
constexpr float SPHERE_BALANCE_KD_MAX            = 2.0f;

// Max velocity correction added to user drive command (both axes, [-1, 1] space).
// Reduce if balance correction overwhelms user drive input during manual control.
constexpr float SPHERE_BALANCE_CORRECTION_MAX    = 0.70f;

// Deadband — suppress corrections when nearly upright to prevent servo jitter.
// Increase if servos chatter at rest; decrease if response feels sluggish near zero.
constexpr float SPHERE_BALANCE_DEADBAND_DEG      = 0.8f;  // deg

// Lean feedforward: desired pitch setpoint = vel_y_commanded * this [deg].
// Leave at 0 until basic PD is stable. Increase to 3–8 to reduce drive-vs-balance
// fighting during sustained motion. At 5°: full-speed forward commands a 5° forward lean.
constexpr float SPHERE_BALANCE_LEAN_DEG_PER_UNIT = 0.0f;

// Reference angular rate for Kd normalization [deg/s].
// rate_norm = pitch_rate_degs / RATE_REF_DEGS.
// 180°/s ≈ half-rotation per second — aggressive but achievable during rocking.
// If derivative correction feels too strong at moderate speeds, increase this value.
constexpr float SPHERE_BALANCE_RATE_REF_DEGS     = 180.0f;

// Sign convention — MUST verify on hardware before enabling any gain (Step 0).
// +1: nose-up pitch (positive error) → negative correction (drive backward to restore).
// Physically: when drive unit lags sphere (nose up), drive backward to reduce the lag.
// If enabling balance INCREASES oscillation, flip to -1 and reflash.
constexpr int SPHERE_BALANCE_PITCH_SIGN = -1;
constexpr int SPHERE_BALANCE_ROLL_SIGN  = 1;