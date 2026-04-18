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
    // 1. Enable accel FIRST
spiWrite8(LSM6DSV::CTRL1, LSM6DSV::ACCEL_ODR_104HZ | LSM6DSV::ACCEL_FS_4G);
delay(50);  // critical: allow accel domain to stabilize

// 2. Then enable gyro
spiWrite8(LSM6DSV::CTRL2, LSM6DSV::GYRO_ODR_104HZ | LSM6DSV::GYRO_FS_500DPS);
delay(50);  // critical: gyro startup time

// Explicitly enable gyro (required on some silicon)
uint8_t ctrl10 = spiRead8(LSM6DSV::CTRL10);
spiWrite8(LSM6DSV::CTRL10, ctrl10 | LSM6DSV::GYRO_ENABLE);
delay(20);

// 3. THEN enable interface features
spiWrite8(LSM6DSV::CTRL3, LSM6DSV::CTRL3_IF_INC | LSM6DSV::CTRL3_BDU);
delay(10);

    // ── Readback verification — paste these values in your bug report ─────────
    uint8_t ctrl1v = spiRead8(LSM6DSV::CTRL1);
    uint8_t ctrl2v = spiRead8(LSM6DSV::CTRL2);
    uint8_t ctrl3v = spiRead8(LSM6DSV::CTRL3);
    Serial.printf("[IMU] CTRL1=0x%02X (expect 0x48)  "
                  "CTRL2=0x%02X (expect 0x48)  "
                  "CTRL3=0x%02X (expect 0x44)\n",
                  ctrl1v, ctrl2v, ctrl3v);

    if (ctrl2v != (LSM6DSV::GYRO_ODR_104HZ | LSM6DSV::GYRO_FS_500DPS)) {
        Serial.printf("[IMU] FATAL: CTRL2 mismatch — gyro ODR not set. "
                      "Got 0x%02X, want 0x48. Check SPI wiring (MOSI/CS).\n", ctrl2v);
        // Do not return false — accel still works; SFLP will degrade gracefully.
    }

    Serial.println("[IMU] LSM6DSV16XTR initialized");

    if (!enableSFLP()) {
        Serial.println("[IMU] WARNING: SFLP enable failed");
    }

// Kick-start gyro pipeline
for (int i = 0; i < 5; i++) {
    uint8_t dummy[6];
    spiBurstRead(LSM6DSV::OUTX_L_G, dummy, 6);
    delay(10);
}

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

// ── TEMPORARY DIAGNOSTIC — remove after gyro confirmed working ────────────
    // If gyro bytes are all 0x00, the ODR is not set (gyro is off).
    // If they are all 0xFF, SPI MISO is floating.
    static uint8_t printCount = 0;
    if (printCount < 20) { // print first 20 reads only
        Serial.printf("[IMU] Raw gyro bytes: %02X %02X %02X %02X %02X %02X | "
                      "accel: %02X %02X %02X %02X %02X %02X\n",
                      buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
                      buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]);
        printCount++;
    }

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

    // Enable embedded function access
    spiWrite8(LSM6DSV::FUNC_CFG_ACCESS, LSM6DSV::EMBEDDED_FUNC_REG_ACCESS);
    delay(2);

    // Enable SFLP
    uint8_t ena = spiRead8(LSM6DSV::EMB_FUNC_EN_A);
    spiWrite8(LSM6DSV::EMB_FUNC_EN_A, ena | LSM6DSV::SFLP_GAME_EN);

    uint8_t verify = spiRead8(LSM6DSV::EMB_FUNC_EN_A);

    spiWrite8(LSM6DSV::FUNC_CFG_ACCESS, 0x00);

    if (!(verify & LSM6DSV::SFLP_GAME_EN)) {
        Serial.println("[IMU] SFLP enable FAILED");
        return false;
    }

    // ── FIFO CONFIG (CRITICAL) ─────────────────────────

    // Set FIFO to continuous mode
    spiWrite8(LSM6DSV::FIFO_CTRL4, LSM6DSV::FIFO_MODE_CONTINUOUS);

    // Enable SFLP batching into FIFO
    spiWrite8(LSM6DSV::FIFO_CTRL2, 0x08); // SFLP batch enable (verify bit if needed)

    delay(20);

    Serial.println("[IMU] SFLP + FIFO enabled");

    return true;
}

SFLPData IMU::readSFLP() {

    SFLPData out{};

    uint8_t tag;
    uint8_t buf[6];

    // Read tag
    spiBurstRead(LSM6DSV::FIFO_DATA_OUT_TAG, &tag, 1);

    uint8_t sensorTag = tag >> 3;

    if (sensorTag != LSM6DSV::FIFO_TAG_SFLP_GAME_ROT) {
        out.valid = false;
        return out;
    }

    // Read quaternion payload
    spiBurstRead(LSM6DSV::FIFO_DATA_OUT_TAG, buf, 6);

    uint16_t hx = (buf[1] << 8) | buf[0];
    uint16_t hy = (buf[3] << 8) | buf[2];
    uint16_t hz = (buf[5] << 8) | buf[4];

    out.qx = float16ToFloat32(hx);
    out.qy = float16ToFloat32(hy);
    out.qz = float16ToFloat32(hz);

    float norm2 = out.qx*out.qx + out.qy*out.qy + out.qz*out.qz;

    if (norm2 >= 1.0f) {
        out.valid = false;
        return out;
    }

    out.qw = sqrtf(1.0f - norm2);
    out.valid = true;

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

void IMU::debugDump() {
    Serial.println("\n========== IMU DEBUG ==========");

    uint8_t who = spiRead8(LSM6DSV::WHO_AM_I_REG);
    Serial.printf("WHO_AM_I: 0x%02X\n", who);

    uint8_t ctrl1 = spiRead8(LSM6DSV::CTRL1);
    uint8_t ctrl2 = spiRead8(LSM6DSV::CTRL2);
    uint8_t ctrl3 = spiRead8(LSM6DSV::CTRL3);
    uint8_t status = spiRead8(LSM6DSV::STATUS_REG);

    Serial.printf("CTRL1: 0x%02X\n", ctrl1);
    Serial.printf("CTRL2: 0x%02X\n", ctrl2);
    Serial.printf("CTRL3: 0x%02X\n", ctrl3);
    Serial.printf("STATUS: 0x%02X  XLDA:%d  GDA:%d\n",
                  status,
                  (status & LSM6DSV::STATUS_XLDA) ? 1 : 0,
                  (status & LSM6DSV::STATUS_GDA) ? 1 : 0);

    uint8_t buf[12];
    spiBurstRead(LSM6DSV::OUTX_L_G, buf, 12);

    Serial.print("RAW BYTES: ");
    for (int i = 0; i < 12; i++) {
        Serial.printf("%02X ", buf[i]);
    }
    Serial.println();

    int16_t gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz = (int16_t)((buf[5] << 8) | buf[4]);

    int16_t ax = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t ay = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t az = (int16_t)((buf[11] << 8) | buf[10]);

    Serial.printf("RAW GYRO:  %d  %d  %d\n", gx, gy, gz);
    Serial.printf("RAW ACCEL: %d  %d  %d\n", ax, ay, az);

    Serial.println("================================\n");
}