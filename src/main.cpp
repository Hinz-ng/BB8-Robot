// module:  main.cpp
// layer:   wiring only — no logic lives here
// purpose: Arduino entry point; connects modules in correct order
// phase:   Phase 3 Step 3 — IMU WHO_AM_I verification only
// date:    2025

#include <Arduino.h>
#include "imu.h"

IMU imu;

void setup() {
    Serial.begin(115200);
    delay(1500); // wait for USB CDC to enumerate on Super Mini

    Serial.println("[BOOT] BB-8 controller starting...");

    if (!imu.begin()) {
        Serial.println("[BOOT] FATAL: IMU init failed. Halting.");
        while (true) { delay(1000); } // halt — nothing works without IMU
    }

    Serial.println("[BOOT] Calibrating gyro — hold robot still...");
    if (!imu.calibrate(200)) {
        Serial.println("[BOOT] WARNING: Calibration failed. Proceeding uncalibrated.");
    }

    Serial.println("[BOOT] Init complete. Entering read loop.");
}

void loop() {
    static uint32_t lastPrint = 0;
    uint32_t now = millis();

    // Read at full IMU rate
    RawIMUData data = imu.read();

    // Print at human-readable rate (5 Hz)
    if (now - lastPrint >= 200) {
        lastPrint = now;

        if (!data.valid) {
            Serial.println("[LOOP] IMU read invalid");
            return;
        }

        // Verification targets:
        //   accel_z ≈ +9.81 m/s² (sensor face-up, Z pointing up)
        //   gyro_x/y/z ≈ 0.0 (stationary)
        Serial.printf("Accel [m/s²] X:% 6.3f  Y:% 6.3f  Z:% 6.3f | "
                      "Gyro [rad/s] X:% 7.4f  Y:% 7.4f  Z:% 7.4f\n",
                      data.accel_x_ms2, data.accel_y_ms2, data.accel_z_ms2,
                      data.gyro_x_rads, data.gyro_y_rads, data.gyro_z_rads);
    }
}