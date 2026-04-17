// module:  main.cpp
// layer:   wiring only — no logic lives here
// purpose: Arduino entry point; connects modules in correct order
// phase:   Phase 3 Step 4 + Phase 7 — IMU read loop + WebSocket telemetry
// date:    2025
//
// WiFi mode: AP-first.
//   The robot always creates its own access point on boot.
//   IP is always 192.168.4.1 — no Serial Monitor needed to find it.
//   SSID: "BB8-Robot"   Password: "bb8robot1"
//
// Loop ordering (must be preserved):
//   1. IMU read          (Layer 1)
//   2. WebComm broadcast (Layer 7) — at WEBCOMM_BROADCAST_HZ
//   3. WebComm update    (Layer 7) — cleanup, every tick

#include <Arduino.h>
#include "imu.h"
#include "webcomm.h"
#include "project_wide_defs.h"
#include "state_estimator.h"
#include "motion_controller.h"
#include <cmath>

// ── Global module instances ───────────────────────────────────────────────────
IMU            imu;
StateEstimator stateEstimator;
MotionController motionController;
WebComm        webcomm;

// ── Interval helper ───────────────────────────────────────────────────────────
static bool every(uint32_t& last, uint32_t now, uint32_t period_ms) {
    if (now - last >= period_ms) { last = now; return true; }
    return false;
}

void setup() {
    Serial.begin(115200);

    // ── Native USB CDC serial wait ────────────────────────────────────────────
    // Super Mini uses native USB CDC (no UART bridge chip). The serial port
    // does not exist until the host opens it. Without this loop, all boot
    // messages fire before Serial Monitor can connect and are lost silently.
    // Times out after 5 s so the robot boots normally with no PC attached.
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 5000)) { delay(10); }
    delay(100); // brief settling after host connection

    Serial.println("\n[BOOT] BB-8 controller starting...");

    // ── Layer 1: IMU ─────────────────────────────────────────────────────────
    if (!imu.begin()) {
        Serial.println("[BOOT] FATAL: IMU init failed. Halting.");
        while (true) { delay(1000); }
    }

    Serial.println("[BOOT] Calibrating gyro — hold robot still for 2 s...");
    if (!imu.calibrate(200)) {
        Serial.println("[BOOT] WARNING: Calibration failed. Proceeding uncalibrated.");
    }

    // ── Layer 3: MotionController ─────────────────────────────────────────────
    if (!motionController.begin()) {
        Serial.println("[BOOT] WARNING: MotionController init failed. Servos disabled.");
    }

    // ── Layer 7: WebComm ─────────────────────────────────────────────────────
    webcomm.setDriveCallback([](float vx, float vy) {
        motionController.setDrive(vx, vy);
    });

    webcomm.setSpeedCallback([](float scale) {
        motionController.setSpeedScale(scale);
        webcomm.setCurrentSpeed(scale);  // keep welcome packet in sync
    });

    // AP-first: pass empty strings → webcomm.begin() goes straight to AP mode.
    // SSID: "BB8-Robot"  Password: "bb8robot1"  IP: always 192.168.4.1
    webcomm.begin("", "");

    Serial.println("[BOOT] WiFi AP started.");
    Serial.println("[BOOT] Connect phone to: BB8-Robot  (pass: bb8robot1)");
    Serial.println("[BOOT] Then open browser to: 192.168.4.1");
}

void loop() {
    static uint32_t lastBroadcast = 0;
    constexpr uint32_t BROADCAST_MS = 1000 / WEBCOMM_BROADCAST_HZ;
    uint32_t now = millis();

    // ── Layer 1: IMU raw read (telemetry only) ────────────────────────────────
    RawIMUData data = imu.read();

    // ── Layer 1: SFLP quaternion read ────────────────────────────────────────
    SFLPData sflp = imu.readSFLP();

    // ── Layer 2: quaternion → pitch/roll ─────────────────────────────────────
    IMUState est = stateEstimator.update(sflp);

    // ── Layer 7: broadcast at WEBCOMM_BROADCAST_HZ ──────────────────────────
    if (every(lastBroadcast, now, BROADCAST_MS)) {
        webcomm.broadcastState(data, est);        // ← was broadcastIMU(data)
    }

    // ── Layer 7: WebSocket cleanup (every tick) ───────────────────────────────
    webcomm.update();

    // ── Debug print at 2 Hz ──────────────────────────────────────────────────
    static uint32_t lastPrint = 0;
    if (every(lastPrint, now, 500)) {
        if (data.valid) {
            Serial.printf("Accel [m/s²] X:%6.3f Y:%6.3f Z:%6.3f | "
                          "Gyro [rad/s] X:%7.4f Y:%7.4f Z:%7.4f | "
                          "Pitch:%6.1f° Roll:%6.1f° | "
                          "Clients:%u\n",
                          data.accel_x_ms2, data.accel_y_ms2, data.accel_z_ms2,
                          data.gyro_x_rads, data.gyro_y_rads, data.gyro_z_rads,
                          est.pitch_deg, est.roll_deg,
                          webcomm.clientCount());
        }
    }
}