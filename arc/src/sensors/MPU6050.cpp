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
#include "math.h"
#include "common/common.hpp"

namespace arc {
namespace sensors {

inline static float gyroFactoryTrim(uint8_t factory_trim)
{
	if (factory_trim == 0)
		return 0.0f;
	return 25.0f * 131.0f * powf(1.046f, factory_trim - 1.0f);
}

inline static float accFactoryTrim(uint8_t factory_trim)
{
	if (factory_trim == 0)
		return 0.0f;
	float exp = (factory_trim - 1.0f) / ((1 << 5) - 2.0f);
	return 4096.0f * 0.34f * powf(0.92f / 0.34f, exp);
}

MPU6050::MPU6050(uint8_t address)
	: m_address{ address }
{
	uint8_t who_am_i;
	read(0x75, &who_am_i, 1);
	// printf("I AM: 0x%X\n", who_am_i);

	uint8_t buf;
	read(0x6B, &buf, 1);
	m_temp_enabled = !(buf & MPU6050_TEMP_DISABLED_BIT);
	m_clk_sel = buf & MPU6050_CLK_SEL_BITS;
	// printf("Temp: %hhu\n", m_temp_enabled);
	// printf("Clk: %hhu\n", m_clk_sel);

	read(0x1C, &buf, 1);
	m_acc_scale_index = (buf & MPU6050_ACC_SCALE_BITS);
	// printf("Acc scale: %hhu\n", m_acc_scale_index);

	switch (m_acc_scale_index) {
	case 0:
		m_acc_scale = 1.0f / (1 << 14);
		break;
	case 1:
		m_acc_scale = 1.0f / (1 << 13);
		break;
	case 2:
		m_acc_scale = 1.0f / (1 << 12);
		break;
	case 3:
		m_acc_scale = 1.0f / (1 << 11);
		break;
	default:
		break;
	}

	// read(0x1B, &buf, 1);
	// m_gyro_scale_index = (buf & MPU6050_GYRO_SCALE_BITS);
	// printf("Gyro scale: %hhu\n", m_gyro_scale_index);

	// read(0x1A, &buf, 1);
	// m_DLPF_conf = (buf & MPU6050_DIGITAL_LOW_PASS_FILTER_BITS);
	// printf("DLPF conf: %hhu\n", m_DLPF_conf);

	self_test();
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

int MPU6050::setAccRange(mpu6050_acc_range_t range)
{
	//TODO: add scale range check

	switch (range) {
	case MPU6050_ACC_RANGE_2G:
		m_acc_scale = 1.0f / (1 << 14);
		printf("Acc range set to 2g\n");
		break;
	case MPU6050_ACC_RANGE_4G:
		m_acc_scale = 1.0f / (1 << 13);
		printf("Acc range set to 4g\n");

		break;
	case MPU6050_ACC_RANGE_8G:
		m_acc_scale = 1.0f / (1 << 12);
		printf("Acc range set to 8g\n");

		break;
	case MPU6050_ACC_RANGE_16G:
		m_acc_scale = 1.0f / (1 << 11);
		printf("Acc range set to 16g\n");

		break;
	default:
		printf("INVALID Acc range\n");
		return -1;
	}

	uint8_t acc_conf;
	read(MPU6050_ACC_CONF_ADDR, &acc_conf, 1);
	acc_conf &= 0b11100111;
	acc_conf |= range << 3;

	uint8_t data[2] = { MPU6050_ACC_CONF_ADDR, acc_conf };
	i2c_write_blocking(i2c_default, m_address, data, 2, false);
	sleep_ms(10);

	return 0;
}

int MPU6050::setGyroRange(mpu6050_gyro_range_t range)
{
	//TODO: add scale range check

	switch (range) {
	case MPU6050_GYRO_RANGE_250:
		m_gyro_scale = 1.0f / (1 << 14);
		printf("Gyro range set to 250\n");
		break;
	case MPU6050_GYRO_RANGE_500:
		m_gyro_scale = 1.0f / (1 << 13);
		printf("Gyro range set to 500\n");

		break;
	case MPU6050_GYRO_RANGE_1000:
		m_gyro_scale = 1.0f / (1 << 12);
		printf("Gyro range set to 1000\n");

		break;
	case MPU6050_GYRO_RANGE_2000:
		m_gyro_scale = 1.0f / (1 << 11);
		printf("Gyro range set to 2000\n");

		break;
	default:
		printf("INVALID Gyro range\n");
		return -1;
	}

	uint8_t gyro_conf;
	read(MPU6050_GYRO_CONF_ADDR, &gyro_conf, 1);
	gyro_conf &= 0b11100111;
	gyro_conf |= range << 3;

	uint8_t data[2] = { MPU6050_GYRO_CONF_ADDR, gyro_conf };
	i2c_write_blocking(i2c_default, m_address, data, 2, false);
	sleep_ms(10);

	return 0;
}

int MPU6050::self_test()
{
	printf("\nPERFORMING SELF TEST\n");

	uint8_t buf[4];
	read(0x0D, buf, 4);

	printf("SELF TEST REGISTERS\n");
	for (uint8_t i = 0; i < 4; i++) {
		printf("0x%02.X: %s\n", 0x0D + i, TO_BIT_STR(buf[i]));
	}
	printf("\n");

	float FT[3];
	int8_t acc_fact_test[3];

	uint8_t reg_10_mask = 0b00110000;
	for (uint8_t i = 0; i < 3; i++) {
		uint8_t acc_fact_test_4_2 = ((buf[i] & 0b11100000) >> 3);
		uint8_t acc_fact_test_1_0 =
			((buf[3] & reg_10_mask) >> (4 - i * 2));
		acc_fact_test[i] = acc_fact_test_4_2 | acc_fact_test_1_0;

		FT[i] = 4096.0f * 0.34f *
			pow(0.92f / 0.34f, (acc_fact_test[i] - 1.0f) / 30.0f);
		reg_10_mask >>= 2;

		printf("[%hhu] AFT %hhd, FT: %f\n", i, acc_fact_test[i], FT[i]);
	}
	printf("\n");

	setAccRange(MPU6050_ACC_RANGE_8G);

	int16_t raw_acc[3];
	float acc[3];
	int16_t selftest_response[3];

	enable_self_test();
	printf("\nSELF TEST ENABLED\n");
	getRawAcc(raw_acc);
	printRawAcc(raw_acc);
	getAcc(acc);
	printAcc(acc);
	for (uint8_t i = 0; i < 3; i++) {
		selftest_response[i] = raw_acc[i];
	}
	sleep_ms(1000);

	disable_self_test();
	printf("\nSELF TEST DISABLED\n");
	getRawAcc(raw_acc);
	printRawAcc(raw_acc);
	getAcc(acc);
	printAcc(acc);

	printf("\nSELF TEST COMPUTE\nSelf Test: [ ");
	for (uint8_t i = 0; i < 3; i++) {
		selftest_response[i] -= raw_acc[i];
		m_acc_self_test[i] = (selftest_response[i] - FT[i]) / FT[i];
		printf("%f, ", m_acc_self_test[i]);
	}
	printf(" ]\n");

	if (abs(m_acc_self_test[0]) > 0.1f || abs(m_acc_self_test[1]) > 0.1f ||
	    abs(m_acc_self_test[2]) > 0.1f) {
		fail = true;
		printf("\n !!! SELF TEST FAILED !!!\n");
		return -1;
	} else {
		printf("\n !!! SELF TEST SUCCEDED !!!\n");
	}

	int8_t gyro_fact_test[3];

	for (uint8_t i = 0; i < 3; i++) {
		gyro_fact_test[i] = (buf[i] & 0b00011111);

		FT[i] = 1392.64f *
			pow(0.92f / 0.34f, (gyro_fact_test[i] - 1.0f) / 30.0f);
		reg_10_mask >>= 2;

		printf("[%hhu] AFT %hhd, FT: %f\n", i, acc_fact_test[i], FT[i]);
	}
	printf("\n");

	return 0;
}

int MPU6050::enable_self_test()
{
	setAccRange(MPU6050_ACC_RANGE_8G);

	uint8_t buf;
	read(0x1C, &buf, 1);

	buf |= MPU6050_X_ACC_SELF_TEST_BIT;
	buf |= MPU6050_Y_ACC_SELF_TEST_BIT;
	buf |= MPU6050_Z_ACC_SELF_TEST_BIT;

	uint8_t data[2] = { 0x1C, buf };
	i2c_write_blocking(i2c_default, m_address, data, 2, false);
	sleep_ms(100);

	return 0;
}

int MPU6050::disable_self_test()
{
	uint8_t buf;
	read(0x1C, &buf, 1);

	buf &= 0b00011111;

	uint8_t data[2] = { 0x1C, buf };
	i2c_write_blocking(i2c_default, m_address, data, 2, false);
	sleep_ms(100);

	return 0;
}

int MPU6050::getAcc(float accel[3])
{
	// TODO: add check on read and input
	uint8_t buf[6];
	read(MPU6050_ACC_X_H_ADDR, buf, 6);

	accel[0] = static_cast<int16_t>((buf[0] << 8) | buf[1]) * m_acc_scale;
	accel[1] = static_cast<int16_t>((buf[2] << 8) | buf[3]) * m_acc_scale;
	accel[2] = static_cast<int16_t>((buf[4] << 8) | buf[5]) * m_acc_scale;

	return 0;
}

void MPU6050::printAcc(float accel[3])
{
	printf("Acc = [%f, %f, %f]\t|.| = %f\n", accel[0], accel[1], accel[2],
	       arc::common::modf(accel));
}

int MPU6050::getRawAcc(int16_t accel[3])
{
	// TODO: add check on read
	uint8_t buf[6];
	read(MPU6050_ACC_X_H_ADDR, buf, 6);

	accel[0] = (buf[0] << 8) | buf[1];
	accel[1] = (buf[2] << 8) | buf[3];
	accel[2] = (buf[4] << 8) | buf[5];

	return 0;
}

void MPU6050::printRawAcc(int16_t accel[3])
{
	printf("Raw acc = [%d, %d, %d]\n", accel[0], accel[1], accel[2]);
}

} // namespace sensors
} // namespace arc
