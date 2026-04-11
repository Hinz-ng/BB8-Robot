// module:  imu_registers.h
// layer:   1 (hardware definitions only — no logic)
// purpose: LSM6DSV16XTR register addresses and configuration masks

#pragma once

namespace LSM6DSV {

// Identity
constexpr uint8_t WHO_AM_I_REG  = 0x0F;
constexpr uint8_t WHO_AM_I_VAL  = 0x70; // expected response

// Control registers
constexpr uint8_t CTRL1         = 0x10; // accel ODR + FS
constexpr uint8_t CTRL2         = 0x11; // gyro  ODR + FS
constexpr uint8_t CTRL3         = 0x12; // BDU, SW_RESET, IF_INC

// Status
constexpr uint8_t STATUS_REG    = 0x1E;
constexpr uint8_t STATUS_XLDA   = 0x01; // accel data ready
constexpr uint8_t STATUS_GDA    = 0x02; // gyro  data ready

// Output registers (burst-readable, 6 bytes each)
constexpr uint8_t OUTX_L_G      = 0x22; // gyro  X low
constexpr uint8_t OUTX_L_A      = 0x28; // accel X low

// CTRL1 masks — accel
constexpr uint8_t ACCEL_ODR_104HZ = 0x40; // bits[7:4] = 0100
constexpr uint8_t ACCEL_FS_4G     = 0x08; // bits[3:2] = 10

// CTRL2 masks — gyro
constexpr uint8_t GYRO_ODR_104HZ  = 0x40;
constexpr uint8_t GYRO_FS_500DPS  = 0x08; // bits[3:2] = 10

// CTRL3 masks
constexpr uint8_t CTRL3_IF_INC    = 0x04; // auto-increment address (required for burst)
constexpr uint8_t CTRL3_BDU       = 0x40; // block data update

// SPI framing
constexpr uint8_t SPI_READ_MASK   = 0x80;

} // namespace LSM6DSV