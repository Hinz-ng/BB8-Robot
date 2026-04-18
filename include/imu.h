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

class IMU {
public:
    bool          begin();
    bool          calibrate(uint16_t samples = 200);
    RawIMUData    read();

    const IMUCalibration& getCalibration() const { return _cal; }

private:
    uint8_t  spiRead8(uint8_t reg);
    void     spiBurstRead(uint8_t reg, uint8_t* buf, uint8_t len);
    void     spiWrite8(uint8_t reg, uint8_t val);

    IMUCalibration _cal{};

    static constexpr float ACCEL_SENS_MS2 = 0.122e-3f * 9.81f;
    static constexpr float GYRO_SENS_RADS = 17.50e-3f * (3.14159265f / 180.0f);
};