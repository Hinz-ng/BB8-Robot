// module:  main.cpp — BODY UNIT (ESP32-S3 Super Mini)
// layer:   wiring only — no logic lives here
// purpose: Arduino entry point: IMU -> StateEstimator -> BalanceController
//          -> MotionController -> ESPNowBody
// date:    2025
//
// Loop order unchanged from previous revision. ESPNowBody replaces WebComm.

#include <Arduino.h>
#include "imu.h"
#include "state_estimator.h"
#include "balance_controller.h"
#include "motion_controller.h"
#include "espnow_body.h"          // replaces webcomm.h
#include "project_wide_defs.h"
#include <cmath>

IMU               imu;
StateEstimator    stateEstimator;
BalanceController balanceController;
MotionController  motionController;
ESPNowBody        espnow;

static volatile float _userVelX = 0.0f;
static volatile float _userVelY = 0.0f;

static bool every(uint32_t& last, uint32_t now, uint32_t ms) {
    if (now - last >= ms) { last = now; return true; } return false;
}
static float clamp1(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 5000)) delay(10);
    delay(100);
    Serial.println("\n[BOOT] BB-8 Body Unit starting...");

    if (!imu.begin()) { Serial.println("[BOOT] FATAL: IMU failed."); while(true) delay(1000); }
    Serial.println("[BOOT] Calibrating gyro — hold still 2 s...");
    if (!imu.calibrate(200)) Serial.println("[BOOT] WARNING: Calibration failed.");

    if (!motionController.begin()) Serial.println("[BOOT] WARNING: MotionController failed.");

    { BalanceConfig cfg; cfg.enabled = false; balanceController.setConfig(cfg); }
    Serial.println("[BOOT] BalanceController ready. Balance DISABLED.");

    // Callback API identical to removed WebComm — only object name changes.
    espnow.setDriveCallback([](float vx, float vy) {
        _userVelX = vx; _userVelY = vy;
    });
    espnow.setSpeedCallback([](float scale) {
        motionController.setSpeedScale(scale);
        espnow.setCurrentSpeed(scale);
    });
    espnow.setBalanceEnableCallback([](bool enabled) {
        BalanceConfig cfg = balanceController.getConfig();
        cfg.enabled = enabled;
        balanceController.setConfig(cfg);
        if (!enabled) {
            balanceController.resetEstop();
            motionController.stop();
            _userVelX = 0.0f; _userVelY = 0.0f;
        }
        Serial.printf("[MAIN] Balance %s\n", enabled ? "ENABLED" : "DISABLED");
    });
    espnow.setBalanceTuneCallback([](float kp, float kd, float kp_roll, float kd_roll) {
        BalanceConfig cfg = balanceController.getConfig();
        cfg.kp_pitch = kp; cfg.kd_pitch = kd;
        cfg.kp_roll  = kp_roll; cfg.kd_roll = kd_roll;
        balanceController.setConfig(cfg);
    });

    if (!espnow.begin()) Serial.println("[BOOT] WARNING: ESP-NOW init failed — running headless");
    Serial.println("[BOOT] Body ready. Waiting for head unit contact...");
}

void loop() {
    static uint32_t lastBroadcast = 0;
    constexpr uint32_t BROADCAST_MS = 1000 / WEBCOMM_BROADCAST_HZ;
    uint32_t now = millis();

    RawIMUData data = imu.read();
    IMUState   est  = stateEstimator.update(data);

    const float userVx = _userVelX, userVy = _userVelY;
    BalanceOutput bal = balanceController.update(est, userVx, userVy);

    if (bal.estop_active) {
        motionController.stop();
    } else {
        motionController.setDrive(clamp1(userVx + bal.correction_vel_x),
                                  clamp1(userVy + bal.correction_vel_y));
    }

    if (every(lastBroadcast, now, BROADCAST_MS))
        espnow.sendTelemetry(data, est, bal);

    espnow.update();

    static uint32_t lastPrint = 0;
    if (every(lastPrint, now, 500) && data.valid) {
        const BalanceConfig& cfg = balanceController.getConfig();
        Serial.printf(
            "Pitch:%6.1f° Roll:%6.1f° | Rate:%6.1f°/s | Corr Y:%+5.2f | "
            "Bal:%s Kp:%.2f Kd:%.2f | Estop:%s | Link:%s\n",
            est.pitch_deg, est.roll_deg, bal.pitch_rate_degs, bal.correction_vel_y,
            cfg.enabled?"ON ":"OFF", cfg.kp_pitch, cfg.kd_pitch,
            bal.estop_active?"YES":"no", espnow.isLinked()?"OK":"LOST"
        );
    }
}