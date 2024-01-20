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
namespace MPU6050 {

inline static float gyroFactoryTrim(uint8_t factory_trim, bool y)
{
	if (factory_trim == 0)
		return 0.0f;
	return (y ? -1.0f : 1.0f) * 25.0f * 131.0f *
	       powf(1.046f, factory_trim - 1.0f);
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
	reset();
	reset_paths();
	wake();
	selfTest();
	sleep();
}

int MPU6050::read(uint8_t reg, uint8_t *buf, size_t bytes)
{
	int ret = 0;

	if (buf == nullptr || bytes <= 0 || reg > WHO_AM_I_ADDR) {
		return -1;
	}

	ret = i2c_write_blocking(i2c_default, m_address, &reg, 1, true);
	if (ret != 1) {
		return -1;
	}

	ret = i2c_read_blocking(i2c_default, m_address, buf, bytes, false);
	if (ret != static_cast<int>(bytes)) {
		return -1;
	}

	return 0;
}

int MPU6050::setAccRange(acc_range_t range)
{
	switch (range) {
	case ACC_RANGE_2G:
		m_acc_scale = 1.0f / (1 << 14);
		break;
	case ACC_RANGE_4G:
		m_acc_scale = 1.0f / (1 << 13);

		break;
	case ACC_RANGE_8G:
		m_acc_scale = 1.0f / (1 << 12);

		break;
	case ACC_RANGE_16G:
		m_acc_scale = 1.0f / (1 << 11);
		break;
	default:
		printf("INVALID Acc range\n");
		return -1;
	}

	uint8_t acc_conf;
	if (read(ACC_CONF_ADDR, &acc_conf, 1) != 0) {
		return -1;
	}
	acc_conf &= 0b11100111; // clear the range bits
	acc_conf |= range << 3; // set the range bits

	uint8_t data[2] = { ACC_CONF_ADDR, acc_conf };
	if (i2c_write_blocking(i2c_default, m_address, data, 2, false) != 2) {
		return -1;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::setGyroRange(gyro_range_t range)
{
	switch (range) {
	case GYRO_RANGE_250:
		m_gyro_scale = 1.0f / (1 << 14);
		break;
	case GYRO_RANGE_500:
		m_gyro_scale = 1.0f / (1 << 13);
		break;
	case GYRO_RANGE_1000:
		m_gyro_scale = 1.0f / (1 << 12);
		break;
	case GYRO_RANGE_2000:
		m_gyro_scale = 1.0f / (1 << 11);
		break;
	default:
		printf("INVALID Gyro range\n");
		return -1;
	}

	if (m_is_gyro_rad) {
		printf("Gyro set to radians\n");
		m_gyro_scale = m_gyro_scale / 180.0f * static_cast<float>(M_PI);
	}

	uint8_t gyro_conf;
	if (read(GYRO_CONF_ADDR, &gyro_conf, 1) != 0) {
		return -1;
	}
	gyro_conf &= 0b11100111; // clear the range bits
	gyro_conf |= range << 3; // set the range bits

	uint8_t data[2] = { GYRO_CONF_ADDR, gyro_conf };
	if (i2c_write_blocking(i2c_default, m_address, data, 2, false) != 2) {
		return -1;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::selfTest()
{
	printf("\nSELF TEST:\n");

#ifdef DEBUG
	uint8_t buf[4];
	if (read(TEST_X_ADDR, buf, 4) != 0) {
		return -1;
	};

	printf("\n======= REGISTERS =======\n");
	for (uint8_t i = 0; i < 4; i++) {
		printf("0x%02.X: %s\n", 0x0D + i, BYTE2STR(buf[i]));
	}
	printf("=========================\n");
#endif

	accSelfTest();
	gyroSelfTest();

	return 0;
}

int MPU6050::accSelfTest()
{
	printf(" - Accelerometer: ");

	uint8_t buf[4];
	float FT[3];
	int8_t acc_fact_test[3];
	uint8_t reg_10_mask = 0b00110000;
	int16_t raw_acc[3];
	int32_t selftest_response[3] = { 0 };

	if (read(TEST_X_ADDR, buf, 4) != 0) {
		goto ERROR;
	};

	for (uint8_t i = 0; i < 3; i++) {
		uint8_t acc_fact_test_4_2 = ((buf[i] & 0b11100000) >> 3);
		uint8_t acc_fact_test_1_0 =
			((buf[3] & reg_10_mask) >> (4 - i * 2));
		acc_fact_test[i] = acc_fact_test_4_2 | acc_fact_test_1_0;

		FT[i] = accFactoryTrim(acc_fact_test[i]);
		reg_10_mask >>= 2;
	}

	enableAccSelfTest();
	for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
		getRawAcc(raw_acc);
		for (uint8_t j = 0; j < 3; j++) {
			selftest_response[j] +=
				static_cast<int32_t>(raw_acc[j]);
		}
		sleep_ms(SELF_TEST_SLEEP);
	}

	disableAccSelfTest();
	for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
		getRawAcc(raw_acc);
		for (uint8_t j = 0; j < 3; j++) {
			selftest_response[j] -=
				static_cast<int32_t>(raw_acc[j]);
		}
		sleep_ms(SELF_TEST_SLEEP);
	}

	for (uint8_t i = 0; i < 3; i++) {
		m_acc_self_test[i] =
			((selftest_response[i] / SELF_TEST_SAMPLES) - FT[i]) /
			FT[i];
	}

	if (abs(m_acc_self_test[0]) > SELF_TEST_ACC_THRESHOLD ||
	    abs(m_acc_self_test[1]) > SELF_TEST_ACC_THRESHOLD ||
	    abs(m_acc_self_test[2]) > SELF_TEST_ACC_THRESHOLD) {
		m_self_test_fail = true;
		goto ERROR;
	} else {
		printf("Self Test Succeded => X: %f, Y: %f, Z: %f\n",
		       m_acc_self_test[0], m_acc_self_test[1],
		       m_acc_self_test[2]);
	}

	return 0;

ERROR:
	printf("Self Test FAILED => X: %f, Y: %f, Z: %f\n", m_acc_self_test[0],
	       m_acc_self_test[1], m_acc_self_test[2]);

	return -1;
}

int MPU6050::gyroSelfTest()
{
	printf(" - Gyroscope: ");

	uint8_t buf[4];
	float FT[3];
	int8_t gyro_fact_test[3];
	int16_t raw_gyro[3];
	int32_t selftest_response[3] = { 0 };

	if (read(TEST_X_ADDR, buf, 4) != 0) {
		goto ERROR;
	};

	for (uint8_t i = 0; i < 3; i++) {
		gyro_fact_test[i] = buf[i] & 0b00011111;
		FT[i] = gyroFactoryTrim(gyro_fact_test[i], i == 1);
	}

	enableGyroSelfTest();
	for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
		getRawGyro(raw_gyro);
		for (uint8_t j = 0; j < 3; j++) {
			selftest_response[j] +=
				static_cast<int32_t>(raw_gyro[j]);
		}
		sleep_ms(SELF_TEST_SLEEP);
	}

	disableGyroSelfTest();
	for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
		getRawGyro(raw_gyro);
		for (uint8_t j = 0; j < 3; j++) {
			selftest_response[j] -=
				static_cast<int32_t>(raw_gyro[j]);
		}
		sleep_ms(SELF_TEST_SLEEP);
	}

	for (uint8_t i = 0; i < 3; i++) {
		m_gyro_self_test[i] =
			((selftest_response[i] / SELF_TEST_SAMPLES) - FT[i]) /
			FT[i];
	}

	if (abs(m_gyro_self_test[0]) > SELF_TEST_ACC_THRESHOLD ||
	    abs(m_gyro_self_test[1]) > SELF_TEST_ACC_THRESHOLD ||
	    abs(m_gyro_self_test[2]) > SELF_TEST_ACC_THRESHOLD) {
		m_self_test_fail = true;
		goto ERROR;
	} else {
		printf("Self Test Succeded => X: %f, Y: %f, Z: %f\n",
		       m_gyro_self_test[0], m_gyro_self_test[1],
		       m_gyro_self_test[2]);
	}

	return 0;

ERROR:
	printf("Self Test FAILED => X: %f, Y: %f, Z: %f\n", m_gyro_self_test[0],
	       m_gyro_self_test[1], m_gyro_self_test[2]);
	return -1;
}

int MPU6050::enableAccSelfTest()
{
	if (setAccRange(ACC_RANGE_8G) != 0) {
		return -1;
	}

	uint8_t buf;
	if (read(ACC_CONF_ADDR, &buf, 1) != 0) {
		return -1;
	}

	buf |= X_ACC_SELF_TEST_BIT;
	buf |= Y_ACC_SELF_TEST_BIT;
	buf |= Z_ACC_SELF_TEST_BIT;

	uint8_t data[2] = { ACC_CONF_ADDR, buf };
	if (i2c_write_blocking(i2c_default, m_address, data, 2, false) != 2) {
		return -1;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::disableAccSelfTest()
{
	uint8_t buf;
	if (read(ACC_CONF_ADDR, &buf, 1) != 0) {
		return -1;
	}

	buf &= 0b00011111; // reset self test bits

	uint8_t data[2] = { ACC_CONF_ADDR, buf };
	if (i2c_write_blocking(i2c_default, m_address, data, 2, false) != 2) {
		return -1;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::enableGyroSelfTest()
{
	if (setGyroRange(GYRO_RANGE_250) != 0) {
		return -1;
	}

	uint8_t buf;
	if (read(GYRO_CONF_ADDR, &buf, 1) != 0) {
		return -1;
	}

	buf |= X_GYRO_SELF_TEST_BIT;
	buf |= Y_GYRO_SELF_TEST_BIT;
	buf |= Z_GYRO_SELF_TEST_BIT;

	uint8_t data[2] = { GYRO_CONF_ADDR, buf };
	if (i2c_write_blocking(i2c_default, m_address, data, 2, false) != 2) {
		return -1;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::disableGyroSelfTest()
{
	uint8_t buf;
	if (read(GYRO_CONF_ADDR, &buf, 1) != 0) {
		return -1;
	}

	buf &= 0b00011111; // reset self test bits

	uint8_t data[2] = { GYRO_CONF_ADDR, buf };
	if (i2c_write_blocking(i2c_default, m_address, data, 2, false) != 2) {
		return -1;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::getAcc(float accel[3])
{
	if (accel == nullptr) {
		return -1;
	}

	uint8_t buf[6];
	if (read(ACC_X_H_ADDR, buf, 6) != 0) {
		return -1;
	}

	accel[0] = static_cast<int16_t>((buf[0] << 8) | buf[1]) * m_acc_scale;
	accel[1] = static_cast<int16_t>((buf[2] << 8) | buf[3]) * m_acc_scale;
	accel[2] = static_cast<int16_t>((buf[4] << 8) | buf[5]) * m_acc_scale;

	return 0;
}

void MPU6050::printAcc(float accel[3])
{
	if (accel == nullptr) {
		return;
	}

	printf("Acc = [%f, %f, %f]\t|.| = %f\n", accel[0], accel[1], accel[2],
	       arc::common::modf(accel));
}

int MPU6050::getRawAcc(int16_t accel[3])
{
	if (accel == nullptr) {
		return -1;
	}

	uint8_t buf[6];
	if (read(ACC_X_H_ADDR, buf, 6) != 0) {
		return -1;
	}

	accel[0] = (buf[0] << 8) | buf[1];
	accel[1] = (buf[2] << 8) | buf[3];
	accel[2] = (buf[4] << 8) | buf[5];

	return 0;
}

void MPU6050::printRawAcc(int16_t accel[3])
{
	if (accel == nullptr) {
		return;
	}

	printf("Raw acc = [%d, %d, %d]\n", accel[0], accel[1], accel[2]);
}

int MPU6050::getGyro(float gyro[3])
{
	if (gyro == nullptr) {
		return -1;
	}

	uint8_t buf[6];
	if (read(GYRO_X_H_ADDR, buf, 6) != 0) {
		return -1;
	}

	gyro[0] = static_cast<int16_t>((buf[0] << 8) | buf[1]) * m_gyro_scale;
	gyro[1] = static_cast<int16_t>((buf[2] << 8) | buf[3]) * m_gyro_scale;
	gyro[2] = static_cast<int16_t>((buf[4] << 8) | buf[5]) * m_gyro_scale;

	return 0;
}

void MPU6050::printGyro(float gyro[3])
{
	if (gyro == nullptr) {
		return;
	}

	printf("Gyro = [%f, %f, %f]\n", gyro[0], gyro[1], gyro[2]);
}

int MPU6050::getRawGyro(int16_t gyro[3])
{
	if (gyro == nullptr) {
		return -1;
	}

	uint8_t buf[6];
	if (read(GYRO_X_H_ADDR, buf, 6) != 0) {
		return -1;
	}

	gyro[0] = (buf[0] << 8) | buf[1];
	gyro[1] = (buf[2] << 8) | buf[3];
	gyro[2] = (buf[4] << 8) | buf[5];

	return 0;
}

void MPU6050::printRawGyro(int16_t gyro[3])
{
	if (gyro == nullptr) {
		return;
	}

	printf("Raw gyro = [%d, %d, %d]\n", gyro[0], gyro[1], gyro[2]);
}

int MPU6050::enableInterrupt()
{
	int ret;

	uint8_t buf[] = { INT_PIN_CFG_ADDR, LATCH_INT_ENABLE_BIT };
	ret = i2c_write_blocking(i2c_default, m_address, buf, 2, false);
	if (ret != 2) {
		return -1;
	}
	sleep_ms(10);

	uint8_t buf2[] = { INT_ENABLE_ADDR, DATA_READY_INT_BIT };
	ret = i2c_write_blocking(i2c_default, m_address, buf2, 2, false);
	if (ret != 2) {
		return -1;
	}
	sleep_ms(10);

	return 0;
}

int MPU6050::getInterruptStatus(uint8_t &int_status)
{
	if (read(INT_STATUS_ADDR, &int_status, 1) != 0) {
		return -1;
	}

	return 0;
}

int MPU6050::reset()
{
	uint8_t pwr_mgmt;
	if (read(PWR_MGMT_1_ADDR, &pwr_mgmt) != 0) {
		return -1;
	}

	int ret;

	uint8_t reset_mask = pwr_mgmt | RESET_BIT;
	uint8_t buf[] = { PWR_MGMT_1_ADDR, reset_mask };
	ret = i2c_write_blocking(i2c_default, m_address, buf, 2, false);
	if (ret != 2) {
		return -1;
	}
	sleep_ms(100);

	return 0;
}

int MPU6050::reset_paths()
{
	int ret;

	uint8_t reset_mask = GYRO_RESET_BIT | ACC_RESET_BIT | TEMP_RESET_BIT;
	uint8_t buf[] = { SIGNAL_PATH_RESET_ADDR, reset_mask };
	ret = i2c_write_blocking(i2c_default, m_address, buf, 2, false);
	if (ret != 2) {
		return -1;
	}
	sleep_ms(100);

	return 0;
}

int MPU6050::sleep()
{
	uint8_t pwr_mgmt;
	if (read(PWR_MGMT_1_ADDR, &pwr_mgmt) != 0) {
		return -1;
	}

	uint8_t sleep_mask = pwr_mgmt & SLEEP_BIT;
	uint8_t buf2[] = { PWR_MGMT_1_ADDR, sleep_mask };
	int ret = i2c_write_blocking(i2c_default, m_address, buf2, 2, false);
	if (ret != 2) {
		return -1;
	}
	sleep_ms(10);

	return 0;
}

int MPU6050::wake()
{
	uint8_t pwr_mgmt;
	if (read(PWR_MGMT_1_ADDR, &pwr_mgmt) != 0) {
		return -1;
	}

	uint8_t wake_mask = pwr_mgmt & ~SLEEP_BIT;
	uint8_t buf2[] = { PWR_MGMT_1_ADDR, wake_mask };
	int ret = i2c_write_blocking(i2c_default, m_address, buf2, 2, false);
	if (ret != 2) {
		return -1;
	}
	sleep_ms(10);

	return 0;
}

int MPU6050::getDLPFConfig(dlpf_t &cfg)
{
	uint8_t dlpf_cfg;
	if (read(CONFIG_ADDR, &dlpf_cfg) != 0) {
		return -1;
	}

	cfg = static_cast<dlpf_t>(dlpf_cfg & DLPF_CFG_BITS);

	return 0;
}

int MPU6050::setDLPFConfig(dlpf_t cfg)
{
	uint8_t dlpf_cfg;
	if (read(CONFIG_ADDR, &dlpf_cfg) != 0) {
		return -1;
	}

	dlpf_cfg &= ~DLPF_CFG_BITS; // clear the dlpf bits
	dlpf_cfg |= cfg; // set the dlpf bits
	uint8_t buf2[] = { CONFIG_ADDR, dlpf_cfg };
	int ret = i2c_write_blocking(i2c_default, m_address, buf2, 2, false);
	if (ret != 2) {
		return -1;
	}
	sleep_ms(10);

	return 0;
}

} // MPU6050
} // namespace sensors
} // namespace arc
