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

#define MPU6050_TEST_X_ADDR 0x0D
#define MPU6050_XA_TEST_4_2_BITS 0xE0
#define MPU6050_XG_TEST_BITS 0x1F

#define MPU6050_TEST_Y_ADDR 0x0E
#define MPU6050_YA_TEST_4_2_BITS 0xE0
#define MPU6050_YG_TEST_BITS 0x1F

#define MPU6050_TEST_Z_ADDR 0x0F
#define MPU6050_ZA_TEST_4_2_BITS 0xE0
#define MPU6050_ZG_TEST_BITS 0x1F

#define MPU6050_TEST_A_ADDR 0x10
#define MPU6050_XA_TEST_1_0_BITS 0x30
#define MPU6050_YA_TEST_1_0_BITS 0x0C
#define MPU6050_ZA_TEST_1_0_BITS 0x02

#define MPU6050_SMPLRT_DIV_ADDR 0x19

#define MPU6050_CONFIG_ADDR 0x1A
#define MPU6050_EXT_SYNC_SET_BITS 0x38
#define MPU6050_DLPF_CFG_BITS 0x7

typedef enum {
	MPU6050_BANDWIDTH_260HZ = 0,
	MPU6050_BANDWIDTH_184HZ = 1,
	MPU6050_BANDWIDTH_94HZ = 2,
	MPU6050_BANDWIDTH_44HZ = 3,
	MPU6050_BANDWIDTH_21HZ = 4,
	MPU6050_BANDWIDTH_10HZ = 5,
	MPU6050_BANDWIDTH_5HZ = 6,
} mpu6050_bandwidth_t;

#define MPU6050_GYRO_CONF_ADDR 0x1B
#define MPU6050_X_GYRO_SELF_TEST_BIT (1 << 7)
#define MPU6050_Y_GYRO_SELF_TEST_BIT (1 << 6)
#define MPU6050_Z_GYRO_SELF_TEST_BIT (1 << 5)
#define MPU6050_GYRO_RANGE_BITS 0x18

typedef enum {
	MPU6050_GYRO_RANGE_250 = 0,
	MPU6050_GYRO_RANGE_500 = 1,
	MPU6050_GYRO_RANGE_1000 = 2,
	MPU6050_GYRO_RANGE_2000 = 3,
} mpu6050_gyro_range_t;

#define MPU6050_ACC_CONF_ADDR 0x1C
#define MPU6050_X_ACC_SELF_TEST_BIT (1 << 7)
#define MPU6050_Y_ACC_SELF_TEST_BIT (1 << 6)
#define MPU6050_Z_ACC_SELF_TEST_BIT (1 << 5)
#define MPU6050_ACC_SCALE_BITS 0x18
#define MPU6050_ACC_DHPF_BITS 0x7

typedef enum {
	MPU6050_ACC_RANGE_2G = 0,
	MPU6050_ACC_RANGE_4G = 1,
	MPU6050_ACC_RANGE_8G = 2,
	MPU6050_ACC_RANGE_16G = 3,
} mpu6050_acc_range_t;

typedef enum {
	MPU6050_ACC_DHPF_DISABLE = 0,
	MPU6050_ACC_DHPF_5_HZ = 1,
	MPU6050_ACC_DHPF_2_5_HZ = 2,
	MPU6050_ACC_DHPF_1_25_HZ = 3,
	MPU6050_ACC_DHPF_0_63_HZ = 4,
	MPU6050_ACC_DHPF_UNUSED = 5,
	MPU6050_ACC_DHPF_HOLD = 6,
} mpu6050_acc_highpass_t;

#define MPU6050_FIFO_EN_ADDR 0x23
#define MPU6050_TEMP_FIFO_ENABLE_BIT (1 << 7)
#define MPU6050_X_GYRO_FIFO_ENABLE_BIT (1 << 6)
#define MPU6050_Y_GYRO_FIFO_ENABLE_BIT (1 << 5)
#define MPU6050_Z_GYRO_FIFO_ENABLE_BIT (1 << 4)
#define MPU6050_ACC_FIFO_ENABLE_BIT (1 << 3)
#define MPU6050_SLV2_FIFO_ENABLE_BIT (1 << 2)
#define MPU6050_SLV1_FIFO_ENABLE_BIT (1 << 1)
#define MPU6050_SLV0_FIFO_ENABLE_BIT (1 << 0)

#define MPU6050_INT_PIN_CFG_ADDR 0x37
#define MPU6050_INT_LEVEL_BIT (1 << 7)
#define MPU6050_INT_OPEN_BIT (1 << 6)
#define MPU6050_LATCH_INT_ENABLE_BIT (1 << 5)
#define MPU6050_INT_CLEAR_ON_READ_BIT (1 << 4)
#define MPU6050_FSYNC_INT_LEVEL_BIT (1 << 3)
#define MPU6050_FSYNC_INT_ENABLE_BIT (1 << 2)
#define MPU6050_I2C_BYPASS_ENABLE_BIT (1 << 1)

#define MPU6050_INT_ENABLE_ADDR 0x3A
#define MPU6050_FIFO_OVERFLOW_BIT (1 << 4)
#define MPU6050_I2C_MASTER_INT_BIT (1 << 3)
#define MPU6050_DATA_READY_INT_BIT (1 << 0)

#define MPU6050_INT_STATUS_ADDR 0x3B
#define MPU6050_FIFO_OVERFLOW_INT_ENABLE_BIT (1 << 4)
#define MPU6050_I2C_MASTER_INT_ENABLE_BIT (1 << 3)
#define MPU6050_DATA_READY_INT_ENABLE_BIT (1 << 0)

#define MPU6050_ACC_X_H_ADDR 0x3B
#define MPU6050_ACC_X_L_ADDR 0x3C
#define MPU6050_ACC_Y_H_ADDR 0x3D
#define MPU6050_ACC_Y_L_ADDR 0x3E
#define MPU6050_ACC_Z_H_ADDR 0x3F
#define MPU6050_ACC_Z_L_ADDR 0x40

#define MPU6050_TEMP_H_ADDR 0x41
#define MPU6050_TEMP_L_ADDR 0x42

#define MPU6050_GYRO_X_H_ADDR 0x43
#define MPU6050_GYRO_X_L_ADDR 0x44
#define MPU6050_GYRO_Y_H_ADDR 0x45
#define MPU6050_GYRO_Y_L_ADDR 0x46
#define MPU6050_GYRO_Z_H_ADDR 0x47
#define MPU6050_GYRO_Z_L_ADDR 0x48

#define MPU6050_SIGNAL_PATH_RESET_ADDR 0x68
#define MPU6050_GYRO_RESET_BIT (1 << 2)
#define MPU6050_ACC_RESET_BIT (1 << 1)
#define MPU6050_TEMP_RESET_BIT (1 << 0)

#define MPU6050_USER_CTRL_ADDR 0x6A
#define MPU6050_FIFO_ENABLED_BIT (1 << 6)
#define MPU6050_I2C_MASTER_ENABLED_BIT (1 << 5)
#define MPU6050_I2C_ENABLED_BIT (1 << 4)
#define MPU6050_FIFO_RESET_BIT (1 << 2)
#define MPU6050_I2C_MASTER_RESET_BIT (1 << 1)
#define MPU6050_SIGNAL_COND_RESET_BIT (1 << 0)

#define MPU6050_PWR_MGMT_1_ADDR 0x6B
#define MPU6050_RESET_BIT (1 << 7)
#define MPU6050_SLEEP_BIT (1 << 6)
#define MPU6050_CYCLE_BIT (1 << 5)
#define MPU6050_TEMP_DISABLED_BIT (1 << 3)
#define MPU6050_CLK_SEL_BITS 0x7

typedef enum {
	MPU6050_INTR_8MHZ = 0,
	MPU6050_PLL_GYROX = 1,
	MPU6050_PLL_GYROY = 2,
	MPU6050_PLL_GYROZ = 3,
	MPU6050_PLL_EXT_32K = 4,
	MPU6050_PLL_EXT_19MHZ = 5,
	MPU6050_STOP = 7,
} mpu6050_clock_t;

#define MPU6050_PWR_MGMT_2_ADDR 0x6C
#define MPU6050_LP_WAKE_CTRL_BITS 0xC0
#define MPU6050_STBY_XA_BIT (1 << 5)
#define MPU6050_STBY_YA_BIT (1 << 4)
#define MPU6050_STBY_ZA_BIT (1 << 3)
#define MPU6050_STBY_XG_BIT (1 << 2)
#define MPU6050_STBY_YG_BIT (1 << 1)
#define MPU6050_STBY_ZG_BIT (1 << 0)

typedef enum {
	MPU6050_CYCLE_1_25_HZ = 0,
	MPU6050_CYCLE_5_HZ = 1,
	MPU6050_CYCLE_20_HZ = 2,
	MPU6050_CYCLE_40_HZ = 3,
} mpu6050_cycle_rate_t;

#define MPU6050_FIFO_COUNT_H_ADDR 0x72

#define MPU6050_FIFO_COUNT_L_ADDR 0x73

#define MPU6050_FIFO_DATA_ADDR 0x74

#define MPU6050_WHO_AM_I_ADDR 0x75
#define MPU6050_WHO_AM_I_BITS 0x7E

class MPU6050 {
    public:
	//TODO: pass i2c instance to constructor
	MPU6050(uint8_t address);

	int read(uint8_t reg, uint8_t *buf, size_t bytes);

	int getRawAcc(int16_t accel[3]);
	int getAcc(float accel[3]);
	void printRawAcc(int16_t accel[3]);
	void printAcc(float accel[3]);
	int setAccRange(mpu6050_acc_range_t range);
	int enableAccSelfTest();
	int disableAccSelfTest();

	int getRawGyro(int16_t gyro[3]);
	int getGyro(float gyro[3]);
	void printRawGyro(int16_t gyro[3]);
	void printGyro(float gyro[3]);
	int setGyroRange(mpu6050_gyro_range_t range);
	int enableGyroSelfTest();
	int disableGyroSelfTest();

	int selfTest();
	int accSelfTest();
	int gyroSelfTest();

    private:
	uint8_t m_address;
	float m_acc_scale{ 1.0 };
	float m_gyro_scale{ 1.0 };
	bool m_temp_enabled{ false };
	uint8_t m_clk_sel{ 0 };
	bool m_FIFO_enabled{ false };
	bool m_I2C_master_mode{ false };
	bool m_I2C_enabled{ false };
	uint8_t m_acc_scale_index{ 0 };
	uint8_t m_gyro_scale_index{ 0 };
	uint8_t m_DLPF_conf{ 0 };
	float m_acc_self_test[3]; // %
	float m_gyro_self_test[3]; // %
	bool fail{ false };
	bool m_is_gyro_rad{ false };
};

} // namespace sensors
} // namespace arc
