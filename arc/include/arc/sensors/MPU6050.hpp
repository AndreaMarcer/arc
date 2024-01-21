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

class MPU6050 {
    public:
	enum class DlpfBW {
		_260Hz = 0,
		_184Hz = 1,
		_94Hz = 2,
		_44Hz = 3,
		_21Hz = 4,
		_10Hz = 5,
		_5Hz = 6,
	};

	enum class GyroRange {
		_250 = 0,
		_500 = 1,
		_1000 = 2,
		_2000 = 3,
	};

	enum class AccRange {
		_2G = 0,
		_4G = 1,
		_8G = 2,
		_16G = 3,
	};

	enum class AccDhpfBW {
		DISABLE = 0,
		_5_HZ = 1,
		_2_5_HZ = 2,
		_1_25_HZ = 3,
		_0_63_HZ = 4,
		UNUSED = 5,
		HOLD = 6,
	};

	enum class ClockSource {
		INTR_8MHZ = 0,
		PLL_GYROX = 1,
		PLL_GYROY = 2,
		PLL_GYROZ = 3,
		PLL_EXT_32K = 4,
		PLL_EXT_19MHZ = 5,
		CLK_STOP = 7,
	};

	enum class CycleRate {
		_1_25_Hz = 0,
		_5_Hz = 1,
		_20_Hz = 2,
		_40_Hz = 3,
	};

	//TODO: pass i2c instance to constructor
	MPU6050(uint8_t address);

	int read(uint8_t reg, uint8_t *buf, size_t bytes = 1);

	int getRawAcc(int16_t accel[3]);
	int getAcc(float accel[3]);
	void printRawAcc(int16_t accel[3]);
	void printAcc(float accel[3]);
	int setAccRange(AccRange range);
	int enableAccSelfTest();
	int disableAccSelfTest();

	int getRawGyro(int16_t gyro[3]);
	int getGyro(float gyro[3]);
	void printRawGyro(int16_t gyro[3]);
	void printGyro(float gyro[3]);
	int setGyroRange(GyroRange range);
	int enableGyroSelfTest();
	int disableGyroSelfTest();
	void setGyroRad();
	void setGyroDeg();

	int getDLPFConfig(DlpfBW &cfg);
	int setDLPFConfig(DlpfBW cfg);

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
	bool m_gyro_in_rad{ false };
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

} // namespace sensors
} // namespace arc
