// module:  main.cpp
// layer:   wiring only — no logic lives here
// purpose: Arduino entry point; connects modules in correct order
// phase:   Phase 3 Step 4 + Phase 7 — IMU read loop + WebSocket telemetry
// date:    2025
//
// Loop ordering (must be preserved):
//   1. IMU read          (Layer 1)
//   2. WebComm broadcast (Layer 7) — at WEBCOMM_BROADCAST_HZ
//   3. WebComm update    (Layer 7) — cleanup, every tick

#include <Arduino.h>
#include "imu.h"
#include "webcomm.h"
#include "project_wide_defs.h"

// ── WiFi credentials — CHANGE THESE ──────────────────────────────────────────
// If connection fails, the robot falls back to AP mode automatically.
// AP SSID: "BB8-Robot"  Password: "bb8robot1"
static constexpr char WIFI_SSID[] = "BB8-Robot";
static constexpr char WIFI_PASS[] = "bb8robot";

// ── Global module instances ───────────────────────────────────────────────────
IMU     imu;
WebComm webcomm;

// ── Interval helper ───────────────────────────────────────────────────────────
// Returns true every <period_ms> milliseconds. Non-blocking.
static bool every(uint32_t& last, uint32_t now, uint32_t period_ms) {
    if (now - last >= period_ms) { last = now; return true; }
    return false;
}

void setup() {
    Serial.begin(115200);
    delay(1500); // wait for USB CDC to enumerate on Super Mini

    Serial.println("\n[BOOT] BB-8 controller starting...");

    // ── Layer 1: IMU ─────────────────────────────────────────────────────────
    if (!imu.begin()) {
        Serial.println("[BOOT] FATAL: IMU init failed. Halting.");
        while (true) { delay(1000); }
    }

    Serial.println("[BOOT] Calibrating gyro — hold robot still for 2 s...");
    if (!imu.calibrate(200)) {
        // Non-fatal: proceed with zero bias; gyro drift will be higher
        Serial.println("[BOOT] WARNING: Calibration failed. Proceeding uncalibrated.");
    }

    // ── Layer 7: WebComm ─────────────────────────────────────────────────────
    // Drive callback registered before begin() so it's ready when first command arrives.
    // servo_control is not yet implemented — log only.
    webcomm.setDriveCallback([](float vx, float vy) {
        // TODO Phase 5: route through motionManager.submit(SOURCE_UI, {vx, vy})
        Serial.printf("[DRIVE] vel_x=%.3f  vel_y=%.3f\n", vx, vy);
    });

    webcomm.begin(WIFI_SSID, WIFI_PASS);

    Serial.printf("[BOOT] IP: %s\n", webcomm.ipAddress().c_str());
    Serial.println("[BOOT] Init complete. Open browser to robot IP.");
}

void loop() {
    static uint32_t lastBroadcast = 0;
    constexpr uint32_t BROADCAST_MS = 1000 / WEBCOMM_BROADCAST_HZ;

    uint32_t now = millis();

    // ── Layer 1: read IMU every tick (full 100 Hz) ───────────────────────────
    RawIMUData data = imu.read();

    if (!data.valid) {
        // Don't spam Serial at 100 Hz — throttle to 1 Hz
        static uint32_t lastWarn = 0;
        if (every(lastWarn, now, 1000)) {
            Serial.println("[LOOP] IMU read invalid");
        }
    }

    // ── Layer 7: broadcast IMU at WEBCOMM_BROADCAST_HZ ──────────────────────
    if (every(lastBroadcast, now, BROADCAST_MS)) {
        webcomm.broadcastIMU(data);
    }

    // ── Layer 7: WebSocket cleanup (must run every tick) ─────────────────────
    webcomm.update();

    // ── Debug print at 2 Hz (Serial monitor sanity check) ────────────────────
    // Remove or ifdef-out when servo control is active to reduce loop jitter
    static uint32_t lastPrint = 0;
    if (every(lastPrint, now, 500)) {
        if (data.valid) {
            Serial.printf("Accel [m/s²] X:%6.3f Y:%6.3f Z:%6.3f | "
                          "Gyro [rad/s] X:%7.4f Y:%7.4f Z:%7.4f | "
                          "Clients:%u\n",
                          data.accel_x_ms2, data.accel_y_ms2, data.accel_z_ms2,
                          data.gyro_x_rads, data.gyro_y_rads, data.gyro_z_rads,
                          webcomm.clientCount());
        }
    }
}