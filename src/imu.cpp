#include "imu.h"
#include "imu_registers.h"
#include "project_wide_defs.h"
#include <Arduino.h>

bool IMU::begin() {
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_IMU_CS);

    delay(10); // power-on startup time (datasheet: ≥1ms)

    uint8_t whoami = spiRead8(LSM6DSV::WHO_AM_I_REG);
    if (whoami != LSM6DSV::WHO_AM_I_VAL) {
        Serial.printf("[IMU] WHO_AM_I mismatch: got 0x%02X, expected 0x%02X\n",
                      whoami, LSM6DSV::WHO_AM_I_VAL);
        return false;
    }

    // Enable auto-increment + block data update
    spiWrite8(LSM6DSV::CTRL3,
              LSM6DSV::CTRL3_IF_INC | LSM6DSV::CTRL3_BDU);

    // Accel: 104 Hz, ±4g
    spiWrite8(LSM6DSV::CTRL1,
              LSM6DSV::ACCEL_ODR_104HZ | LSM6DSV::ACCEL_FS_4G);

    // Gyro: 104 Hz, ±500 dps
    spiWrite8(LSM6DSV::CTRL2,
              LSM6DSV::GYRO_ODR_104HZ | LSM6DSV::GYRO_FS_500DPS);

    delay(20); // ODR startup settling
    Serial.println("[IMU] LSM6DSV16XTR initialized OK");
    return true;
}

bool IMU::calibrate(uint16_t samples) {
    // Robot must be stationary during this window
    _cal = {}; // reset
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

    // Burst read gyro first (OUTX_L_G = 0x22, 6 bytes), then accel
    // With IF_INC enabled, address auto-increments across the 12 bytes
    spiBurstRead(LSM6DSV::OUTX_L_G, buf, 12);
    out.valid = true;

    // Gyro (little-endian int16)
    int16_t gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz = (int16_t)((buf[5] << 8) | buf[4]);

    // Accel
    int16_t ax = (int16_t)((buf[7]  << 8) | buf[6]);
    int16_t ay = (int16_t)((buf[9]  << 8) | buf[8]);
    int16_t az = (int16_t)((buf[11] << 8) | buf[10]);

    // Apply sensitivity + bias
    out.gyro_x_rads = gx * GYRO_SENS_RADS - _cal.gyro_bias_x_rads;
    out.gyro_y_rads = gy * GYRO_SENS_RADS - _cal.gyro_bias_y_rads;
    out.gyro_z_rads = gz * GYRO_SENS_RADS - _cal.gyro_bias_z_rads;
    out.accel_x_ms2 = ax * ACCEL_SENS_MS2;
    out.accel_y_ms2 = ay * ACCEL_SENS_MS2;
    out.accel_z_ms2 = az * ACCEL_SENS_MS2;

    return out;
}

// ── SPI primitives ──────────────────────────────────────────────────────────
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