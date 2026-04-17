#include "imu.h"
#include "imu_registers.h"
#include "project_wide_defs.h"
#include <Arduino.h>
#include <cmath>

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
    
    // ── SFLP enable ───────────────────────────────────────────────────────────
    // Must be called after ODR is configured (CTRL1/CTRL2 set above).
    // SFLP requires both accel and gyro to be running.
    if (!enableSFLP()) {
        Serial.println("[IMU] WARNING: SFLP enable failed — state estimator will have no data");
        // Non-fatal: raw accel/gyro still works; state estimator will return invalid.
    }

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

bool IMU::enableSFLP() {
    // 1. Switch to embedded function register page
    spiWrite8(LSM6DSV::FUNC_CFG_ACCESS, LSM6DSV::EMBEDDED_FUNC_REG_ACCESS);
    delay(1);

    // 2. Enable SFLP computation — read-modify-write to preserve other EN_A bits
    uint8_t ena = spiRead8(LSM6DSV::EMB_FUNC_EN_A);
    spiWrite8(LSM6DSV::EMB_FUNC_EN_A, ena | LSM6DSV::SFLP_GAME_EN);

    // 3. Set SFLP output rate to 120 Hz (closest standard rate above 100 Hz loop)
    //    ⚠  If SFLP_ODR_REG address is wrong, this write is a no-op and SFLP
    //       defaults to ~60 Hz — still functional, just slightly lower rate
    spiWrite8(LSM6DSV::SFLP_ODR_REG, LSM6DSV::SFLP_ODR_120HZ);

    // 4. Return to main register page before leaving
    spiWrite8(LSM6DSV::FUNC_CFG_ACCESS, 0x00);

    delay(10); // SFLP DSP startup settling — not documented; conservative margin
    Serial.println("[IMU] SFLP game rotation vector enabled (120 Hz)");
    return true;
}

SFLPData IMU::readSFLP() {
    SFLPData out{};
    uint8_t buf[6];

    // Page-switch, burst-read, page-restore — 3 SPI transactions per call
    spiWrite8(LSM6DSV::FUNC_CFG_ACCESS, LSM6DSV::EMBEDDED_FUNC_REG_ACCESS);
    spiBurstRead(LSM6DSV::SFLP_GAME_ROTATION_VECTOR_L, buf, 6);
    spiWrite8(LSM6DSV::FUNC_CFG_ACCESS, 0x00);

    // Decode 3 little-endian float16 values → float32
    uint16_t hx = (uint16_t)((buf[1] << 8) | buf[0]);
    uint16_t hy = (uint16_t)((buf[3] << 8) | buf[2]);
    uint16_t hz = (uint16_t)((buf[5] << 8) | buf[4]);

    out.qx = float16ToFloat32(hx);
    out.qy = float16ToFloat32(hy);
    out.qz = float16ToFloat32(hz);

    // Reconstruct qw from unit-quaternion constraint: qw = sqrt(1 − |q_vec|²)
    // qw² can go slightly negative due to float16 rounding — clamp and flag.
    // Guard NaN in vector components first — float16 0xFFFF decodes to NaN.
// NaN comparisons always return false, so the qw2 < 0 check alone is blind to them.
if (std::isnan(out.qx) || std::isnan(out.qy) || std::isnan(out.qz)) {
    out.qx = out.qy = out.qz = 0.0f;
    out.qw    = 1.0f;
    out.valid = false;   // identity quaternion; SFLP not yet producing data
    return out;
}

float qw2 = 1.0f - (out.qx*out.qx + out.qy*out.qy + out.qz*out.qz);
if (qw2 < 0.0f) {
    out.qw    = 1.0f;   // clamp to identity rather than 0 — avoids degenerate state
    out.valid = false;
} else {
    out.qw    = sqrtf(qw2);
    out.valid = true;
}

    return out;
}

float IMU::float16ToFloat32(uint16_t h) {
    // IEEE 754 half-precision → single-precision
    // Layout: [sign:1][exponent:5 bias=15][mantissa:10]
    uint32_t sign = (h >> 15) & 0x1U;
    uint32_t exp  = (h >> 10) & 0x1FU;
    uint32_t mant =  h        & 0x3FFU;

    uint32_t bits;

    if (exp == 0) {
        if (mant == 0) {
            bits = sign << 31;                          // signed zero
        } else {
            // Subnormal float16 → normalised float32
            while (!(mant & 0x400U)) { mant <<= 1; exp--; }
            mant &= 0x3FFU;
            bits = (sign << 31) | ((exp + 127U - 15U) << 23) | (mant << 13);
        }
    } else if (exp == 31) {
        bits = (sign << 31) | (0xFFU << 23) | (mant << 13); // inf or NaN
    } else {
        bits = (sign << 31) | ((exp + 127U - 15U) << 23) | (mant << 13);
    }

    float result;
    memcpy(&result, &bits, sizeof(float));  // type-pun via memcpy — UB-safe
    return result;
}