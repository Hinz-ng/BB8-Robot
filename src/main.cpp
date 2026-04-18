// module:  main.cpp
// layer:   wiring only — no logic lives here
// purpose: Arduino entry point; connects modules in correct order
// phase:   Phase 3 Step 4 + Phase 7 + Balance Controller
// date:    2025
//
// WiFi mode: AP-first.
//   SSID: "BB8-Robot"   Password: "bb8robot1"   IP: 192.168.4.1
//
// Loop ordering (must be preserved):
//   1. IMU read                         (Layer 1)
//   2. State estimator update           (Layer 2)
//   3. Balance controller update        (Layer 3) — uses last user vel snapshot
//   4. Mix balance correction + user cmd → setDrive()   (Layer 3)
//   5. E-stop check                     (Layer 3)
//   6. WebComm broadcast                (Layer 7) — at WEBCOMM_BROADCAST_HZ
//   7. WebComm update / WS cleanup      (Layer 7) — every tick

#include <Arduino.h>
#include "imu.h"
#include "state_estimator.h"
#include "balance_controller.h"
#include "motion_controller.h"
#include "webcomm.h"
#include "project_wide_defs.h"
#include <cmath>

// ── Global module instances ───────────────────────────────────────────────────
IMU               imu;
StateEstimator    stateEstimator;
BalanceController balanceController;
MotionController  motionController;
WebComm           webcomm;

// ── Latest user drive command (written by WebComm callback, read by control loop) ──
// The drive callback fires asynchronously from the WebSocket ISR context.
// Both reads and writes are float-aligned on ESP32 — single-instruction, safe
// without a mutex at this access pattern. Do not add heap-allocated structures here.
static volatile float _userVelX = 0.0f;
static volatile float _userVelY = 0.0f;

// ── Interval helper ───────────────────────────────────────────────────────────
static bool every(uint32_t& last, uint32_t now, uint32_t period_ms) {
    if (now - last >= period_ms) { last = now; return true; }
    return false;
}

// ── Clamp helper ─────────────────────────────────────────────────────────────
static float clamp1(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }

void setup() {
    Serial.begin(115200);

    // ── Native USB CDC serial wait ────────────────────────────────────────────
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 5000)) { delay(10); }
    delay(100);

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

    // ── Layer 3: BalanceController ────────────────────────────────────────────
    // Balance starts DISABLED. Enable via UI after sign verification (Step 0).
    // See project_wide_defs.h SPHERE_BALANCE_PITCH_SIGN comment and tuning guide.
    {
        BalanceConfig cfg;
        cfg.enabled = false;
        balanceController.setConfig(cfg);
    }
    Serial.println("[BOOT] BalanceController ready. Balance DISABLED — enable after sign check.");

    // ── Layer 7: WebComm ─────────────────────────────────────────────────────

    // Drive callback — snapshot user command for the control loop.
    // Balance correction is added in the loop, not here, to maintain the
    // phase relationship: read IMU → estimate → balance → mix → drive.
    webcomm.setDriveCallback([](float vx, float vy) {
        _userVelX = vx;
        _userVelY = vy;
    });

    webcomm.setSpeedCallback([](float scale) {
        motionController.setSpeedScale(scale);
        webcomm.setCurrentSpeed(scale);
    });

    // Balance enable/disable — update config in place, preserve current gains.
    webcomm.setBalanceEnableCallback([](bool enabled) {
        BalanceConfig cfg = balanceController.getConfig();
        cfg.enabled = enabled;
        balanceController.setConfig(cfg);
        if (!enabled) {
            // Clear any latched e-stop so robot doesn't stay stopped
            // unexpectedly when the operator toggles balance off after recovery.
            balanceController.resetEstop();
            // Zero drive on disable so the robot doesn't lurch from a
            // non-zero user command that was being held during the toggle.
            motionController.stop();
            _userVelX = 0.0f;
            _userVelY = 0.0f;
        }
        Serial.printf("[MAIN] Balance %s\n", enabled ? "ENABLED" : "DISABLED");
    });

    // Balance tune — clamping already done in WebComm::_parseCommand.
    webcomm.setBalanceTuneCallback([](float kp, float kd, float kp_roll, float kd_roll) {
        BalanceConfig cfg = balanceController.getConfig();
        cfg.kp_pitch = kp;
        cfg.kd_pitch = kd;
        cfg.kp_roll  = kp_roll;
        cfg.kd_roll  = kd_roll;
        balanceController.setConfig(cfg);
        // No Serial print — webcomm.cpp already logs the values.
    });

    webcomm.begin("", "");

    Serial.println("[BOOT] WiFi AP started.");
    Serial.println("[BOOT] Connect phone to: BB8-Robot  (pass: bb8robot1)");
    Serial.println("[BOOT] Then open browser to: 192.168.4.1");
    Serial.println("[BOOT] REMINDER: Open Balance panel → verify sign BEFORE enabling.");
}

void loop() {
    static uint32_t lastBroadcast = 0;
    constexpr uint32_t BROADCAST_MS = 1000 / WEBCOMM_BROADCAST_HZ;
    uint32_t now = millis();

    // ── Layer 1: IMU raw read ─────────────────────────────────────────────────
    RawIMUData data = imu.read();

    // ── Layer 2: State estimator ──────────────────────────────────────────────
    IMUState est = stateEstimator.update(data);

    // ── Layer 3: Balance controller ───────────────────────────────────────────
    // Snapshot volatile user commands once per tick for consistency.
    const float userVx = _userVelX;
    const float userVy = _userVelY;

    BalanceOutput bal = balanceController.update(est, userVx, userVy);

    // ── E-stop: stop servos immediately, skip drive mixing ───────────────────
    // E-stop is latched in BalanceController and auto-clears at 80% of threshold.
    // It fires even when balance is disabled — purely a safety gate.
    if (bal.estop_active) {
        motionController.stop();
        // Do not call setDrive() this tick.
        // Fall through to broadcast + update so the UI keeps showing telemetry.
    } else {
        // ── Drive mixing: user command + balance correction ───────────────────
        // Mix AFTER balance computes — balance correction is an additive
        // overlay on user intent, not a replacement.
        // MotionController::setDrive() internally clamps to [-1, +1],
        // so overflow from large corrections is harmless.
        //
        // When balance is disabled (bal.correction_vel_* == 0.0f), this
        // is identical to the pre-balance drive path.
        const float mixedVx = clamp1(userVx + bal.correction_vel_x);
        const float mixedVy = clamp1(userVy + bal.correction_vel_y);
        motionController.setDrive(mixedVx, mixedVy);
    }

    // ── Layer 7: broadcast at WEBCOMM_BROADCAST_HZ ──────────────────────────
    if (every(lastBroadcast, now, BROADCAST_MS)) {
        webcomm.broadcastState(data, est, bal);
    }

    // ── Layer 7: WebSocket cleanup (every tick) ───────────────────────────────
    webcomm.update();

    // ── Debug print at 2 Hz ──────────────────────────────────────────────────
    static uint32_t lastPrint = 0;
    if (every(lastPrint, now, 500)) {
        if (data.valid) {
            const BalanceConfig& cfg = balanceController.getConfig();
            Serial.printf(
                "Pitch:%6.1f° Roll:%6.1f° | Rate:%6.1f°/s | "
                "Corr Y:%+5.2f | Bal:%s Kp:%.2f Kd:%.2f | "
                "Estop:%s | Clients:%u\n",
                est.pitch_deg, est.roll_deg,
                bal.pitch_rate_degs,
                bal.correction_vel_y,
                cfg.enabled  ? "ON " : "OFF",
                cfg.kp_pitch, cfg.kd_pitch,
                bal.estop_active ? "YES" : "no",
                webcomm.clientCount()
            );
        }
    }
}