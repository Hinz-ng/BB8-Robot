// module:  espnow_protocol.h
// layer:   transport — shared message definitions for head↔body ESP-NOW link
// purpose: packed structs and MsgType enum used by both MCUs.
// CRITICAL: IDENTICAL COPY required in both head-unit and robot-controller.
//           Any struct change → update both copies → reflash both units.
// Struct sizes (packed): BodyTelemetry = 58 bytes, HeadCommand = 22 bytes.
//   Both are well under the 250-byte ESP-NOW MTU.
// date:    2025

#pragma once
#include <cstdint>

// ── Message type discriminator ────────────────────────────────────────────────
enum class MsgType : uint8_t {
    TELEMETRY  = 0x01,  // body → head: full IMU + balance state
    DRIVE      = 0x02,  // head → body: vel_x, vel_y
    SPEED      = 0x03,  // head → body: speed scale
    STOP       = 0x04,  // head → body: emergency stop
    BAL_ENABLE = 0x05,  // head → body: enable/disable balance controller
    BAL_TUNE   = 0x06,  // head → body: update PD gains live
    PING       = 0x10,  // either direction: link health probe
    PONG       = 0x11,  // reply to PING
};

// ── Body → Head telemetry ─────────────────────────────────────────────────────
// Sent at WEBCOMM_BROADCAST_HZ (20 Hz). Contains everything the WebSocket UI
// previously received from broadcastState(RawIMUData, IMUState, BalanceOutput).
// Packed size: 58 bytes.
struct __attribute__((packed)) BodyTelemetry {
    uint8_t  msg_type;              // always MsgType::TELEMETRY

    // Layer 1 — raw IMU (bias-corrected)
    float    accel_x_ms2;           // m/s²
    float    accel_y_ms2;
    float    accel_z_ms2;
    float    gyro_x_rads;           // rad/s
    float    gyro_y_rads;
    float    gyro_z_rads;

    // Layer 2 — complementary filter output
    float    pitch_deg;             // deg — positive = nose up
    float    roll_deg;              // deg — positive = right side down

    // Layer 3 — balance controller output
    float    pitch_rate_degs;       // deg/s — Kd input, already converted in balance_controller.cpp
    float    correction_vel_y;      // [-1, +1] — balance correction on pitch axis
    float    correction_vel_x;      // [-1, +1] — balance correction on roll axis

    // System
    uint32_t uptime_ms;
    uint16_t seq_num;               // rolling counter; head uses to detect dropped packets

    bool     imu_valid;
    bool     bal_enabled;
    bool     estop_active;
};

static_assert(sizeof(BodyTelemetry) <= 250,
    "BodyTelemetry exceeds 250-byte ESP-NOW MTU — reduce fields");

// ── Head → Body command ───────────────────────────────────────────────────────
// Event-driven — one packet per user action (joystick, slider, toggle).
// Field interpretation depends on msg_type (see comment per MsgType):
//   DRIVE:      f1=vel_x,    f2=vel_y
//   SPEED:      f1=scale
//   BAL_ENABLE: u1=enabled   (0 or 1)
//   BAL_TUNE:   f1=kp_pitch, f2=kd_pitch, f3=kp_roll, f4=kd_roll
//   STOP / PING / PONG: all params unused
// Packed size: 22 bytes.
struct __attribute__((packed)) HeadCommand {
    uint8_t  msg_type;
    float    f1;
    float    f2;
    float    f3;
    float    f4;
    uint8_t  u1;
};

static_assert(sizeof(HeadCommand) <= 250,
    "HeadCommand exceeds 250-byte ESP-NOW MTU");