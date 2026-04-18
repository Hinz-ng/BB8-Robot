#include "imu.h"
#include "imu_registers.h"
#include "project_wide_defs.h"
#include <Arduino.h>
#include <cmath>

bool IMU::begin() {
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_IMU_CS);
    delay(10); // power-on startup

    // ── WHO_AM_I ─────────────────────────────────────────────────────────────
    uint8_t whoami = spiRead8(LSM6DSV::WHO_AM_I_REG);
    if (whoami != LSM6DSV::WHO_AM_I_VAL) {
        Serial.printf("[IMU] WHO_AM_I mismatch: got 0x%02X, expected 0x%02X\n",
                      whoami, LSM6DSV::WHO_AM_I_VAL);
        return false;
    }
    Serial.printf("[IMU] WHO_AM_I OK: 0x%02X\n", whoami);

    // ── Software reset ────────────────────────────────────────────────────────
    // Required before any configuration. Without this, ODR writes can silently
    // fail on cold boot. SW_RESET bit auto-clears when reset is complete.
    spiWrite8(LSM6DSV::CTRL3, 0x01); // bit 0 = SW_RESET
    delay(50); // datasheet: max reset time 50 ms
    // Verify reset cleared — bit 0 should be 0 now
    uint8_t ctrl3_post_reset = spiRead8(LSM6DSV::CTRL3);
    if (ctrl3_post_reset & 0x01) {
        Serial.println("[IMU] WARNING: SW_RESET did not clear — chip may be unresponsive");
    }
    Serial.printf("[IMU] SW_RESET complete. CTRL3=0x%02X\n", ctrl3_post_reset);

    // ── Configure ─────────────────────────────────────────────────────────────
    // CTRL3 first — IF_INC must be set before burst reads work correctly.
    spiWrite8(LSM6DSV::CTRL3, LSM6DSV::CTRL3_IF_INC | LSM6DSV::CTRL3_BDU);
    spiWrite8(LSM6DSV::CTRL1, LSM6DSV::ACCEL_ODR_104HZ | LSM6DSV::ACCEL_FS_4G);
    spiWrite8(LSM6DSV::CTRL2, LSM6DSV::GYRO_ODR_104HZ  | LSM6DSV::GYRO_FS_500DPS);
    delay(20); // ODR startup settling

    // ── Readback verification ─────────────────────────────────────────────────
    uint8_t ctrl1v = spiRead8(LSM6DSV::CTRL1);
    uint8_t ctrl2v = spiRead8(LSM6DSV::CTRL2);
    uint8_t ctrl3v = spiRead8(LSM6DSV::CTRL3);
    Serial.printf("[IMU] CTRL1=0x%02X (expect 0x48)  "
                  "CTRL2=0x%02X (expect 0x48)  "
                  "CTRL3=0x%02X (expect 0x44)\n",
                  ctrl1v, ctrl2v, ctrl3v);

    if (ctrl2v != (LSM6DSV::GYRO_ODR_104HZ | LSM6DSV::GYRO_FS_500DPS)) {
        Serial.printf("[IMU] WARNING: CTRL2 mismatch — gyro ODR not set. "
                      "Got 0x%02X, want 0x48. Check SPI wiring (MOSI/CS).\n", ctrl2v);
    }

    Serial.println("[IMU] LSM6DSV16XTR initialized");
    return true;
}

bool IMU::calibrate(uint16_t samples) {
    _cal = {};
    float bx = 0, by = 0, bz = 0;

    for (uint16_t i = 0; i < samples; ++i) {
        RawIMUData d = read();
        if (!d.valid) { Serial.println("[IMU] Calibration read failed"); return false; }
        bx += d.gyro_x_rads;
        by += d.gyro_y_rads;
        bz += d.gyro_z_rads;
        delay(1000 / IMU_LOOP_HZ);
    }

    _cal.gyro_bias_x_rads = bx / samples;
    _cal.gyro_bias_y_rads = by / samples;
    _cal.gyro_bias_z_rads = bz / samples;
    _cal.calibrated = true;

    Serial.printf("[IMU] Gyro bias — X:%.5f Y:%.5f Z:%.5f rad/s\n",
                  _cal.gyro_bias_x_rads, _cal.gyro_bias_y_rads, _cal.gyro_bias_z_rads);
    return true;
}

RawIMUData IMU::read() {
    RawIMUData out{};
    uint8_t buf[12]; // 6 gyro bytes + 6 accel bytes

    spiBurstRead(LSM6DSV::OUTX_L_G, buf, 12);
    out.valid = true;

    int16_t gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz = (int16_t)((buf[5] << 8) | buf[4]);

    int16_t ax = (int16_t)((buf[7]  << 8) | buf[6]);
    int16_t ay = (int16_t)((buf[9]  << 8) | buf[8]);
    int16_t az = (int16_t)((buf[11] << 8) | buf[10]);

    out.gyro_x_rads = gx * GYRO_SENS_RADS - _cal.gyro_bias_x_rads;
    out.gyro_y_rads = gy * GYRO_SENS_RADS - _cal.gyro_bias_y_rads;
    out.gyro_z_rads = gz * GYRO_SENS_RADS - _cal.gyro_bias_z_rads;
    out.accel_x_ms2 = ax * ACCEL_SENS_MS2;
    out.accel_y_ms2 = ay * ACCEL_SENS_MS2;
    out.accel_z_ms2 = az * ACCEL_SENS_MS2;

    return out;
}

// ── SPI primitives ────────────────────────────────────────────────────────────
void IMU::spiBurstRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transfer(reg | LSM6DSV::SPI_READ_MASK);
    for (uint8_t i = 0; i < len; ++i) buf[i] = SPI.transfer(0x00);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
}

uint8_t IMU::spiRead8(uint8_t reg) {
    uint8_t val;
    spiBurstRead(reg, &val, 1);
    return val;
}

void IMU::spiWrite8(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE3));
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transfer(reg & ~LSM6DSV::SPI_READ_MASK);
    SPI.transfer(val);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
}