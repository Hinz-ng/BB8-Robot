// module:  imu.h
// layer:   1 — raw hardware I/O only
// purpose: LSM6DSV16XTR SPI driver; outputs RawIMUData
// inputs:  SPI bus, CS pin (from project_wide_defs.h)
// outputs: RawIMUData struct (accel m/s², gyro rad/s, validity flag)
// deps:    imu_registers.h, project_wide_defs.h

#pragma once
#include <SPI.h>
#include <cstdint>

struct RawIMUData {
    float accel_x_ms2;  // m/s²
    float accel_y_ms2;
    float accel_z_ms2;
    float gyro_x_rads;  // rad/s
    float gyro_y_rads;
    float gyro_z_rads;
    bool  valid;        // false if SPI transaction failed or WHO_AM_I mismatch
};

struct IMUCalibration {
    float gyro_bias_x_rads;  // rad/s — collected at boot standstill
    float gyro_bias_y_rads;
    float gyro_bias_z_rads;
    bool  calibrated;
};

struct SFLPData {
    float qw;    // reconstructed scalar component: sqrt(1 − qx²−qy²−qz²)
    float qx;    // x vector component
    float qy;    // y vector component
    float qz;    // z vector component
    bool  valid; // false if SPI failed or qw² < 0 (float16 rounding edge case)
};

class IMU {
public:
    bool          begin();   // init SPI, verify WHO_AM_I, configure ODR/FS
    bool          calibrate(uint16_t samples = 200); // gyro bias collection
    RawIMUData    read();    // burst-read, apply bias, return calibrated data

    const IMUCalibration& getCalibration() const { return _cal; }

    bool     enableSFLP();  // called once inside begin() — do not call separately
    SFLPData readSFLP();    // burst-read 6 bytes, decode float16, reconstruct qw

private:
    uint8_t  spiRead8(uint8_t reg);
    void     spiBurstRead(uint8_t reg, uint8_t* buf, uint8_t len);
    void     spiWrite8(uint8_t reg, uint8_t val);

    // Decodes IEEE 754 half-precision (float16) to float32.
    // SFLP game rotation vector components are stored as float16 in hardware.
    static float float16ToFloat32(uint16_t h);

    IMUCalibration _cal{};

    // Sensitivity constants (datasheet Table 2)
    // FS=±4g  → 0.122 mg/LSB → 0.122e-3 * 9.81 m/s²/LSB
    static constexpr float ACCEL_SENS_MS2 = 0.122e-3f * 9.81f;
    // FS=±500dps → 17.50 mdps/LSB → * π/180 / 1000
    static constexpr float GYRO_SENS_RADS = 17.50e-3f * (3.14159265f / 180.0f);
};