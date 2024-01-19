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
namespace MPU6050 {

constexpr uint8_t TEST_X_ADDR = 0x0D;
constexpr uint8_t XA_TEST_4_2_BITS = 0xE0;
constexpr uint8_t XG_TEST_BITS = 0x1F;

constexpr uint8_t TEST_Y_ADDR = 0x0E;
constexpr uint8_t YA_TEST_4_2_BITS = 0xE0;
constexpr uint8_t YG_TEST_BITS = 0x1F;

constexpr uint8_t TEST_Z_ADDR = 0x0F;
constexpr uint8_t ZA_TEST_4_2_BITS = 0xE0;
constexpr uint8_t ZG_TEST_BITS = 0x1F;

constexpr uint8_t TEST_A_ADDR = 0x10;
constexpr uint8_t XA_TEST_1_0_BITS = 0x30;
constexpr uint8_t YA_TEST_1_0_BITS = 0x0C;
constexpr uint8_t ZA_TEST_1_0_BITS = 0x02;

constexpr uint8_t SMPLRT_DIV_ADDR = 0x19;

constexpr uint8_t CONFIG_ADDR = 0x1A;
constexpr uint8_t EXT_SYNC_SET_BITS = 0x38;
constexpr uint8_t DLPF_CFG_BITS = 0x7;

typedef enum {
	DLPF_260HZ = 0,
	DLPF_184HZ = 1,
	DLPF_94HZ = 2,
	DLPF_44HZ = 3,
	DLPF_21HZ = 4,
	DLPF_10HZ = 5,
	DLPF_5HZ = 6,
} dlpf_t;

constexpr uint8_t GYRO_CONF_ADDR = 0x1B;
constexpr uint8_t X_GYRO_SELF_TEST_BIT = (1 << 7);
constexpr uint8_t Y_GYRO_SELF_TEST_BIT = (1 << 6);
constexpr uint8_t Z_GYRO_SELF_TEST_BIT = (1 << 5);
constexpr uint8_t GYRO_RANGE_BITS = 0x18;

typedef enum {
	GYRO_RANGE_250 = 0,
	GYRO_RANGE_500 = 1,
	GYRO_RANGE_1000 = 2,
	GYRO_RANGE_2000 = 3,
} gyro_range_t;

constexpr uint8_t ACC_CONF_ADDR = 0x1C;
constexpr uint8_t X_ACC_SELF_TEST_BIT = (1 << 7);
constexpr uint8_t Y_ACC_SELF_TEST_BIT = (1 << 6);
constexpr uint8_t Z_ACC_SELF_TEST_BIT = (1 << 5);
constexpr uint8_t ACC_SCALE_BITS = 0x18;
constexpr uint8_t ACC_DHPF_BITS = 0x7;

typedef enum {
	ACC_RANGE_2G = 0,
	ACC_RANGE_4G = 1,
	ACC_RANGE_8G = 2,
	ACC_RANGE_16G = 3,
} acc_range_t;

typedef enum {
	ACC_DHPF_DISABLE = 0,
	ACC_DHPF_5_HZ = 1,
	ACC_DHPF_2_5_HZ = 2,
	ACC_DHPF_1_25_HZ = 3,
	ACC_DHPF_0_63_HZ = 4,
	ACC_DHPF_UNUSED = 5,
	ACC_DHPF_HOLD = 6,
} acc_dhpfs_t;

constexpr uint8_t FIFO_EN_ADDR = 0x23;
constexpr uint8_t TEMP_FIFO_ENABLE_BIT = (1 << 7);
constexpr uint8_t X_GYRO_FIFO_ENABLE_BIT = (1 << 6);
constexpr uint8_t Y_GYRO_FIFO_ENABLE_BIT = (1 << 5);
constexpr uint8_t Z_GYRO_FIFO_ENABLE_BIT = (1 << 4);
constexpr uint8_t ACC_FIFO_ENABLE_BIT = (1 << 3);
constexpr uint8_t SLV2_FIFO_ENABLE_BIT = (1 << 2);
constexpr uint8_t SLV1_FIFO_ENABLE_BIT = (1 << 1);
constexpr uint8_t SLV0_FIFO_ENABLE_BIT = (1 << 0);

constexpr uint8_t INT_PIN_CFG_ADDR = 0x37;
/**
 * @brief When this bit is equal to 0, the logic level for the INT pin is active high. 
 * When this bit is equal to 1, the logic level for the INT pin is active low. 
 */
constexpr uint8_t INT_LEVEL_BIT = (1 << 7);
/**
 * @brief When this bit is equal to 0, the INT pin is configured as push-pull.
 * When this bit is equal to 1, the INT pin is configured as open drain.
 */
constexpr uint8_t INT_OPEN_BIT = (1 << 6);
/**
 * @brief When this bit is equal to 0, the INT pin emits a 50us long pulse.
 * When this bit is equal to 1, the INT pin is held high until the interrupt is cleared
 */
constexpr uint8_t LATCH_INT_ENABLE_BIT = (1 << 5);
/**
 * @brief When this bit is equal to 0, interrupt status bits are cleared only by reading 
 * INT_STATUS (Register 58).
 * When this bit is equal to 1, interrupt status bits are cleared on any read operation.
 */
constexpr uint8_t INT_CLEAR_ON_READ_BIT = (1 << 4);
constexpr uint8_t FSYNC_INT_LEVEL_BIT = (1 << 3);
constexpr uint8_t FSYNC_INT_ENABLE_BIT = (1 << 2);
constexpr uint8_t I2C_BYPASS_ENABLE_BIT = (1 << 1);

constexpr uint8_t INT_ENABLE_ADDR = 0x38;
constexpr uint8_t FIFO_OVERFLOW_BIT = (1 << 4);
constexpr uint8_t I2C_MASTER_INT_BIT = (1 << 3);
/**
 * @brief When set to 1, this bit enables the Data Ready interrupt, which occurs each 
 * time a write operation to all of the sensor registers has been completed.
 */
constexpr uint8_t DATA_READY_INT_BIT = (1 << 0);

constexpr uint8_t INT_STATUS_ADDR = 0x3A;
constexpr uint8_t FIFO_OVERFLOW_INT_ENABLE_BIT = (1 << 4);
constexpr uint8_t I2C_MASTER_INT_ENABLE_BIT = (1 << 3);
/**
 * @brief This bit automatically sets to 1 when a Data Ready interrupt is generated. 
 * The bit clears to 0 after the register has been read.
 */
constexpr uint8_t DATA_READY_INT_ENABLE_BIT = (1 << 0);

constexpr uint8_t ACC_X_H_ADDR = 0x3B;
constexpr uint8_t ACC_X_L_ADDR = 0x3C;
constexpr uint8_t ACC_Y_H_ADDR = 0x3D;
constexpr uint8_t ACC_Y_L_ADDR = 0x3E;
constexpr uint8_t ACC_Z_H_ADDR = 0x3F;
constexpr uint8_t ACC_Z_L_ADDR = 0x40;

constexpr uint8_t TEMP_H_ADDR = 0x41;
constexpr uint8_t TEMP_L_ADDR = 0x42;

constexpr uint8_t GYRO_X_H_ADDR = 0x43;
constexpr uint8_t GYRO_X_L_ADDR = 0x44;
constexpr uint8_t GYRO_Y_H_ADDR = 0x45;
constexpr uint8_t GYRO_Y_L_ADDR = 0x46;
constexpr uint8_t GYRO_Z_H_ADDR = 0x47;
constexpr uint8_t GYRO_Z_L_ADDR = 0x48;

constexpr uint8_t SIGNAL_PATH_RESET_ADDR = 0x68;
constexpr uint8_t GYRO_RESET_BIT = (1 << 2);
constexpr uint8_t ACC_RESET_BIT = (1 << 1);
constexpr uint8_t TEMP_RESET_BIT = (1 << 0);

constexpr uint8_t USER_CTRL_ADDR = 0x6A;
constexpr uint8_t FIFO_ENABLED_BIT = (1 << 6);
constexpr uint8_t I2C_MASTER_ENABLED_BIT = (1 << 5);
constexpr uint8_t I2C_ENABLED_BIT = (1 << 4);
constexpr uint8_t FIFO_RESET_BIT = (1 << 2);
constexpr uint8_t I2C_MASTER_RESET_BIT = (1 << 1);
constexpr uint8_t SIGNAL_COND_RESET_BIT = (1 << 0);

constexpr uint8_t PWR_MGMT_1_ADDR = 0x6B;
constexpr uint8_t RESET_BIT = (1 << 7);
constexpr uint8_t SLEEP_BIT = (1 << 6);
constexpr uint8_t CYCLE_BIT = (1 << 5);
constexpr uint8_t TEMP_DISABLED_BIT = (1 << 3);
constexpr uint8_t CLK_SEL_BITS = 0x7;

typedef enum {
	INTR_8MHZ = 0,
	PLL_GYROX = 1,
	PLL_GYROY = 2,
	PLL_GYROZ = 3,
	PLL_EXT_32K = 4,
	PLL_EXT_19MHZ = 5,
	CLK_STOP = 7,
} clock_t;

constexpr uint8_t PWR_MGMT_2_ADDR = 0x6C;
constexpr uint8_t LP_WAKE_CTRL_BITS = 0xC0;
constexpr uint8_t STBY_XA_BIT = (1 << 5);
constexpr uint8_t STBY_YA_BIT = (1 << 4);
constexpr uint8_t STBY_ZA_BIT = (1 << 3);
constexpr uint8_t STBY_XG_BIT = (1 << 2);
constexpr uint8_t STBY_YG_BIT = (1 << 1);
constexpr uint8_t STBY_ZG_BIT = (1 << 0);

typedef enum {
	CYCLE_1_25_HZ = 0,
	CYCLE_5_HZ = 1,
	CYCLE_20_HZ = 2,
	CYCLE_40_HZ = 3,
} cycle_rate_t;

constexpr uint8_t FIFO_COUNT_H_ADDR = 0x72;

constexpr uint8_t FIFO_COUNT_L_ADDR = 0x73;

constexpr uint8_t FIFO_DATA_ADDR = 0x74;

constexpr uint8_t WHO_AM_I_ADDR = 0x75;
constexpr uint8_t WHO_AM_I_BITS = 0x7E;

constexpr uint8_t SELF_TEST_SAMPLES = 20;
constexpr uint8_t SELF_TEST_SLEEP = 10;
constexpr uint8_t SELF_TEST_ACC_THRESHOLD = 0.1;

class MPU6050 {
    public:
	//TODO: pass i2c instance to constructor
	MPU6050(uint8_t address);

	int read(uint8_t reg, uint8_t *buf, size_t bytes = 1);

	int getRawAcc(int16_t accel[3]);
	int getAcc(float accel[3]);
	void printRawAcc(int16_t accel[3]);
	void printAcc(float accel[3]);
	int setAccRange(acc_range_t range);
	int enableAccSelfTest();
	int disableAccSelfTest();

	int getRawGyro(int16_t gyro[3]);
	int getGyro(float gyro[3]);
	void printRawGyro(int16_t gyro[3]);
	void printGyro(float gyro[3]);
	int setGyroRange(gyro_range_t range);
	int enableGyroSelfTest();
	int disableGyroSelfTest();

	int getDLPFConfig(dlpf_t &cfg);
	int setDLPFConfig(dlpf_t cfg);

	int selfTest();
	int accSelfTest();
	int gyroSelfTest();

	int enableInterrupt();
	int getInterruptStatus(uint8_t &int_status);

	int reset();
	int reset_paths();
	int sleep();
	int wake();

    private:
	bool m_is_gyro_rad{ false };
	uint8_t m_address{ 0 };
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
	float m_acc_self_test[3];
	float m_gyro_self_test[3];
	bool m_self_test_fail{ false };
};

} // MPU6050
} // namespace sensors
} // namespace arc
