/**
 * @file MPU6050.hpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-01-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "pico/stdlib.h"
#include "common/constants.hpp"
#include "common/common.hpp"

namespace arc {
namespace sensors {

// Register 6B (107)
#define MPU6050_RESET_BIT (1 << 7)
#define MPU6050_SLEEP_BIT (1 << 6)
#define MPU6050_CYCLE_BIT (1 << 5)
#define MPU6050_TEMP_DISABLED_BIT (1 << 3)
#define MPU6050_CLK_SEL_BITS 0x7

// Register 6A (106)
#define MPU6050_FIFO_ENABLED_BIT (1 << 6)
#define MPU6050_I2C_MASTER_ENABLED_BIT (1 << 5)
#define MPU6050_I2C_ENABLED_BIT (1 << 4)
#define MPU6050_FIFO_RESET_BIT (1 << 2)
#define MPU6050_I2C_MASTER_RESET_BIT (1 << 1)
#define MPU6050_SIGNAL_COND_RESET_BIT (1 << 0)

// Register 68 (104)
#define MPU6050_GYRO_RESET_BIT (1 << 2)
#define MPU6050_ACC_RESET_BIT (1 << 1)
#define MPU6050_TEMP_RESET_BIT (1 << 0)

// GYRO
#define MPU6050_x_GYRO_H_REG 43
#define MPU6050_x_GYRO_L_REG 44
#define MPU6050_Y_GYRO_H_REG 45
#define MPU6050_Y_GYRO_L_REG 46
#define MPU6050_Z_GYRO_H_REG 47
#define MPU6050_Z_GYRO_L_REG 48

// TEMP
#define MPU6050_TEMP_H_REG 41
#define MPU6050_TEMP_L_REG 42

// ACC
#define MPU6050_X_ACC_H_REG 3B
#define MPU6050_X_ACC_L_REG 3C
#define MPU6050_Y_ACC_H_REG 3D
#define MPU6050_Y_ACC_L_REG 3E
#define MPU6050_Z_ACC_H_REG 3F
#define MPU6050_Z_ACC_L_REG 40

// Register 3A (58)
#define MPU6050_FIFO_OVERFLOW_BIT (1 << 4)
#define MPU6050_I2C_MASTER_INT_BIT (1 << 3)
#define MPU6050_DATA_READY_INT_BIT (1 << 0)

// Register 3B (56)
#define MPU6050_FIFO_OVERFLOW_INT_ENABLE_BIT (1 << 4)
#define MPU6050_I2C_MASTER_INT_ENABLE_BIT (1 << 3)
#define MPU6050_DATA_READY_INT_ENABLE_BIT (1 << 0)

// Register 37 (55)
#define MPU6050_INT_LEVEL_BIT (1 << 7)
#define MPU6050_INT_OPEN_BIT (1 << 6)
#define MPU6050_LATCH_INT_ENABLE_BIT (1 << 5)
#define MPU6050_INT_CLEAR_ON_READ_BIT (1 << 4)
#define MPU6050_FSYNC_INT_LEVEL_BIT (1 << 3)
#define MPU6050_FSYNC_INT_ENABLE_BIT (1 << 2)
#define MPU6050_I2C_BYPASS_ENABLE_BIT (1 << 1)

// Register 36 (54)
// #define MPU6050__BIT (1 << 7)
// #define MPU6050__BIT (1 << 6)
// #define MPU6050__BIT (1 << 5)
// #define MPU6050__BIT (1 << 4)
// #define MPU6050__BIT (1 << 3)
// #define MPU6050__BIT (1 << 2)
// #define MPU6050__BIT (1 << 1)
// #define MPU6050__BIT (1 << 0)

// Register 23 (35)
#define MPU6050_TEMP_FIFO_ENABLE_BIT (1 << 7)
#define MPU6050_X_GYRO_FIFO_ENABLE_BIT (1 << 6)
#define MPU6050_Y_GYRO_FIFO_ENABLE_BIT (1 << 5)
#define MPU6050_Z_GYRO_FIFO_ENABLE_BIT (1 << 4)
#define MPU6050_ACC_FIFO_ENABLE_BIT (1 << 3)
#define MPU6050_SLV2_FIFO_ENABLE_BIT (1 << 2)
#define MPU6050_SLV1_FIFO_ENABLE_BIT (1 << 1)
#define MPU6050_SLV0_FIFO_ENABLE_BIT (1 << 0)

// Register 1C (28)
#define MPU6050_X_ACC_SELF_TEST_BIT (1 << 7)
#define MPU6050_Y_ACC_SELF_TEST_BIT (1 << 6)
#define MPU6050_Z_ACC_SELF_TEST_BIT (1 << 5)
#define MPU6050_ACC_SCALE_BITS 0x18

#define MPU6050_ACC_SCALE_2G 0x0
#define MPU6050_ACC_SCALE_4G 0x1
#define MPU6050_ACC_SCALE_8G 0x2
#define MPU6050_ACC_SCALE_16G 0x3

// Register 1B (27)
#define MPU6050_X_GYRO_SELF_TEST_BIT (1 << 7)
#define MPU6050_Y_GYRO_SELF_TEST_BIT (1 << 6)
#define MPU6050_Z_GYRO_SELF_TEST_BIT (1 << 5)
#define MPU6050_GYRO_SCALE_BITS 0x18

#define MPU6050_GYRO_SCALE_250 0x0
#define MPU6050_GYRO_SCALE_500 0x1
#define MPU6050_GYRO_SCALE_1000 0x2
#define MPU6050_GYRO_SCALE_2000 0x3

// Register 1A (26)
#define MPU6050_EXTERNAL_SYNC_BITS 0x56
#define MPU6050_DIGITAL_LOW_PASS_FILTER_BITS 0x7

// Register  ()
// #define MPU6050__BIT (1 << 7)
// #define MPU6050__BIT (1 << 6)
// #define MPU6050__BIT (1 << 5)
// #define MPU6050__BIT (1 << 4)
// #define MPU6050__BIT (1 << 3)
// #define MPU6050__BIT (1 << 2)
// #define MPU6050__BIT (1 << 1)
// #define MPU6050__BIT (1 << 0)

class MPU6050 {
    public:
	MPU6050(uint8_t address);
	float m_acc_scale{ 1.0 };

	int read(uint8_t reg, uint8_t *buf, size_t bytes);

	int get_raw_acc(int16_t accel[3]);
	void print_raw_acc(int16_t accel[3]);
	int get_acc(float accel[3]);
	void print_acc(float accel[3]);

	int set_acc_scale(uint8_t scale);
	int self_test();

	int enable_self_test();
	int disable_self_test();

    private:
	uint8_t m_address;

	bool m_temp_enabled{ false };
	uint8_t m_clk_sel{ 0 };
	bool m_FIFO_enabled{ false };
	bool m_I2C_master_mode{ false };
	bool m_I2C_enabled{ false };
	uint8_t m_acc_scale_index{ 0 };
	uint8_t m_gyro_scale_index{ 0 };
	uint8_t m_DLPF_conf{ 0 };
	float m_acc_self_test[3]; // %
	bool fail{ false };
};

} // namespace sensors
} // namespace arc
