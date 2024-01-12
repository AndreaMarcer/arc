/**
 * @file MPU6050.cpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdio.h>
#include "sensors/MPU6050.hpp"
#include "hardware/i2c.h"

namespace arc {
namespace sensors {

MPU6050::MPU6050(uint8_t address)
	: m_address{ address }
{
	uint8_t who_am_i;
	read(0x75, &who_am_i, 1);
	printf("I AM: 0x%X\n", who_am_i);

	uint8_t buf;
	read(0x6B, &buf, 1);
	m_temp_enabled = !(buf & MPU6050_TEMP_DISABLED_BIT);
	m_clk_sel = buf & MPU6050_CLK_SEL_BITS;
	printf("Temp: %hhu\n", m_temp_enabled);
	printf("Clk: %hhu\n", m_clk_sel);

	read(0x1C, &buf, 1);
	m_acc_scale_index = !(buf & MPU6050_ACC_SCALE_BITS);
	printf("Acc scale: %hhu\n", m_acc_scale_index);

	switch (m_acc_scale_index) {
	case 0:
		m_acc_scale = 2.0f * 2.0f * arc::common::k_g / (1 << 16);
		break;
	case 1:
		m_acc_scale = 2.0f * 4.0f * arc::common::k_g / (1 << 16);
		break;
	case 2:
		m_acc_scale = 2.0f * 8.0f * arc::common::k_g / (1 << 16);
		break;
	case 3:
		m_acc_scale = 2.0f * 16.0f * arc::common::k_g / (1 << 16);
		break;
	default:
		break;
	}

	read(0x1B, &buf, 1);
	m_gyro_scale_index = !(buf & MPU6050_GYRO_SCALE_BITS);
	printf("Gyro scale: %hhu\n", m_gyro_scale_index);

	read(0x1A, &buf, 1);
	m_DLPF_conf = !(buf & MPU6050_DIGITAL_LOW_PASS_FILTER_BITS);
	printf("Gyro scale: %hhu\n", m_DLPF_conf);
}

int MPU6050::read(uint8_t reg, uint8_t *buf, size_t bytes)
{
	if (buf == nullptr || bytes <= 0 || reg > 0x75) {
		return -1;
	}

	if (i2c_write_blocking(i2c_default, m_address, &reg, 1, true) != 1) {
		return -1;
	}

	if (i2c_read_blocking(i2c_default, m_address, buf, bytes, false) !=
	    (int)bytes) {
		return -1;
	}

	return 0;
}

int MPU6050::set_acc_scale(uint8_t scale)
{
	uint8_t accel_config;

	//TODO: add scale range check

	switch (scale) {
	case 0:
		m_acc_scale = 2.0f * 2.0f * arc::common::k_g / (1 << 16);
		break;
	case 1:
		m_acc_scale = 2.0f * 4.0f * arc::common::k_g / (1 << 16);
		break;
	case 2:
		m_acc_scale = 2.0f * 8.0f * arc::common::k_g / (1 << 16);
		break;
	case 3:
		m_acc_scale = 2.0f * 16.0f * arc::common::k_g / (1 << 16);
		break;
	default:
		break;
	}

	read(0x1C, &accel_config, 1);
	accel_config = (scale << 3);

	uint8_t data[2] = { 0x1C, accel_config };
	i2c_write_blocking(i2c_default, m_address, data, 2, false);

	return 0;
}

} // namespace sensors
} // namespace arc
