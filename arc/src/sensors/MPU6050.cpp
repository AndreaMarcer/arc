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
#include "math.h"
#include "sensors/MPU6050.hpp"
#include "hardware/i2c.h"
#include "common/common.hpp"
#include "common/log.hpp"
#include <errno.h>

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
		log_error("Invalid argument\n");
		return EINVAL;
	}

	ret = i2c_write_blocking(i2c_default, m_address, &reg, 1, true);
	if (ret != 1) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	ret = i2c_read_blocking(i2c_default, m_address, buf, bytes, false);
	if (ret != static_cast<int>(bytes)) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	return 0;
}

int MPU6050::setAccRange(acc_range_t range)
{
	if (range < ACC_RANGE_2G || range > ACC_RANGE_16G) {
		log_error("Invalid argument\n");
		return EINVAL;
	}

	int ret = 0;

	uint8_t acc_conf;
	ret = read(ACC_CONF_ADDR, &acc_conf, 1);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	acc_conf &= 0b11100111; // clear the range bits
	acc_conf |= range << 3; // set the range bits
	uint8_t data[2] = { ACC_CONF_ADDR, acc_conf };
	ret = i2c_write_blocking(i2c_default, m_address, data, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	switch (range) {
	case ACC_RANGE_2G:
		m_acc_scale = 1.0f / (1 << 14);
		log_debug("Acceleration scale set to 2g: %f\n", m_acc_scale);
		break;
	case ACC_RANGE_4G:
		m_acc_scale = 1.0f / (1 << 13);
		log_debug("Acceleration scale set to 4g: %f\n", m_acc_scale);
		break;
	case ACC_RANGE_8G:
		m_acc_scale = 1.0f / (1 << 12);
		log_debug("Acceleration scale set to 8g: %f\n", m_acc_scale);
		break;
	case ACC_RANGE_16G:
		m_acc_scale = 1.0f / (1 << 11);
		log_debug("Acceleration scale set to 16g: %f\n", m_acc_scale);
		break;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::setGyroRange(gyro_range_t range)
{
	if (range < GYRO_RANGE_250 || range > GYRO_RANGE_2000) {
		log_error("Invalid argument\n");
		return EINVAL;
	}

	int ret = 0;
	uint8_t gyro_conf;
	ret = read(GYRO_CONF_ADDR, &gyro_conf, 1);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	gyro_conf &= 0b11100111; // clear the range bits
	gyro_conf |= range << 3; // set the range bits
	uint8_t data[2] = { GYRO_CONF_ADDR, gyro_conf };
	ret = i2c_write_blocking(i2c_default, m_address, data, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	switch (range) {
	case GYRO_RANGE_250:
		m_gyro_scale = 1.0f / (1 << 14);
		log_debug("Gyroscope scale set to 250°: %f\n", m_gyro_scale);
		break;
	case GYRO_RANGE_500:
		m_gyro_scale = 1.0f / (1 << 13);
		log_debug("Gyroscope scale set to 500°: %f\n", m_gyro_scale);
		break;
	case GYRO_RANGE_1000:
		m_gyro_scale = 1.0f / (1 << 12);
		log_debug("Gyroscope scale set to 1000°: %f\n", m_gyro_scale);
		break;
	case GYRO_RANGE_2000:
		m_gyro_scale = 1.0f / (1 << 11);
		log_debug("Gyroscope scale set to 2000°: %f\n", m_gyro_scale);
		break;
	}

	if (m_gyro_in_rad) {
		m_gyro_scale *= arc::common::DEG2RAD;
		log_debug("Gyro scale set to radians: %f\n", m_gyro_scale);
	}

	sleep_ms(10);

	return 0;
}

void MPU6050::setGyroRad()
{
	if (!m_gyro_in_rad) {
		m_gyro_scale *= arc::common::DEG2RAD;
		m_gyro_in_rad = true;
		log_debug("Gyro scale set to radians: %f\n", m_gyro_scale);
	} else {
		log_debug("Gyro already set to radians: %f\n", m_gyro_scale);
	}
}

void MPU6050::setGyroDeg()
{
	if (m_gyro_in_rad) {
		m_gyro_scale *= arc::common::RAD2DEG;
		m_gyro_in_rad = false;
		log_debug("Gyro scale set to degrees: %f\n", m_gyro_scale);
	} else {
		log_debug("Gyro already set to degrees: %f\n", m_gyro_scale);
	}
}

int MPU6050::selfTest()
{
	log_info("\n");
	log_info("SELF TEST:\n");

	int ret = 0;

#ifdef LOG_DEBUG
	uint8_t buf[4];
	ret = read(TEST_X_ADDR, buf, 4);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	};

	log_debug(" - REGISTERS:\n");
	for (uint8_t i = 0; i < 4; i++) {
		log_debug("   - 0x%02.X: %s\n", 0x0D + i, BYTE2STR(buf[i]));
	}
#endif

	ret = accSelfTest();
	if (ret != 0) {
		log_error("Error in accSelfTest(). (%s)\n", strerror(ret));
		return ret;
	}

	ret = gyroSelfTest();
	if (ret != 0) {
		log_error("Error in gyroSelfTest(). (%s)\n", strerror(ret));
		return ret;
	}

	return m_self_test_fail;
}

int MPU6050::accSelfTest()
{
	log_info(" - Accelerometer:\n");

	uint8_t buf[4];
	float FT[3];
	int8_t acc_fact_test[3];
	uint8_t reg_10_mask = 0b00110000;
	int16_t raw_acc[3];
	int32_t selftest_response[3] = { 0 };

	int ret = 0;
	ret = read(TEST_X_ADDR, buf, 4);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		goto FAIL;
	}

	log_debug("   - FT: [");
	for (uint8_t i = 0; i < 3; i++) {
		uint8_t acc_fact_test_4_2 = ((buf[i] & 0b11100000) >> 3);
		uint8_t acc_fact_test_1_0 =
			((buf[3] & reg_10_mask) >> (4 - i * 2));
		acc_fact_test[i] = acc_fact_test_4_2 | acc_fact_test_1_0;

		FT[i] = accFactoryTrim(acc_fact_test[i]);
		log_debug_s("%f, ", FT[i]);
		reg_10_mask >>= 2;
	}
	log_debug_s("]\n");

	log_debug("   - Self Test \n");
	ret = enableAccSelfTest();
	if (ret != 0) {
		log_error("Error in enableAccSelfTest(). (%s)\n",
			  strerror(ret));
		goto FAIL;
	}
	log_debug("     - Enabled\n");
	for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
		ret = getRawAcc(raw_acc);
		if (ret != 0) {
			log_error("Error in getRawAcc(). (%s)\n",
				  strerror(ret));
			goto FAIL;
		}
		for (uint8_t j = 0; j < 3; j++) {
			selftest_response[j] +=
				static_cast<int32_t>(raw_acc[j]);
		}
		sleep_ms(SELF_TEST_SLEEP);
	}

	ret = disableAccSelfTest();
	if (ret != 0) {
		log_error("Error in disableAccSelfTest(). (%s)\n",
			  strerror(ret));
		goto FAIL;
	}
	log_debug("     - Disabled\n");
	for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
		ret = getRawAcc(raw_acc);
		if (ret != 0) {
			log_error("Error in getRawAcc(). (%s)\n",
				  strerror(ret));
			goto FAIL;
		}
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

	if (abs(m_acc_self_test[0]) < SELF_TEST_ACC_THRESHOLD &&
	    abs(m_acc_self_test[1]) < SELF_TEST_ACC_THRESHOLD &&
	    abs(m_acc_self_test[2]) < SELF_TEST_ACC_THRESHOLD) {
		log_info("   - [%0.3f, %0.3f, %0.3f] => PASSED\n",
			 m_acc_self_test[0], m_acc_self_test[1],
			 m_acc_self_test[2]);
		return 0;
	}

	log_warning("   - [%0.3f, %0.3f, %0.3f] => FAILED\n",
		    m_acc_self_test[0], m_acc_self_test[1], m_acc_self_test[2]);

FAIL:
	m_self_test_fail = true;
	return ret;
}

int MPU6050::gyroSelfTest()
{
	log_info(" - Gyroscope:\n");

	uint8_t buf[4];
	float FT[3];
	int8_t gyro_fact_test[3];
	int16_t raw_gyro[3];
	int32_t selftest_response[3] = { 0 };

	int ret = 0;
	ret = read(TEST_X_ADDR, buf, 4);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		goto FAIL;
	}

	log_debug("   - FT: [");
	for (uint8_t i = 0; i < 3; i++) {
		gyro_fact_test[i] = buf[i] & 0b00011111;
		FT[i] = gyroFactoryTrim(gyro_fact_test[i], i == 1);
		log_debug_s("%f, ", FT[i]);
	}
	log_debug_s("]\n");

	log_debug("   - Self Test \n");
	ret = enableGyroSelfTest();
	if (ret != 0) {
		log_error("Error in enableGyroSelfTest(). (%s)\n",
			  strerror(ret));
		goto FAIL;
	}
	log_debug("     - Enabled\n");
	for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
		ret = getRawGyro(raw_gyro);
		if (ret != 0) {
			log_error("Error in getRawGyro(). (%s)\n",
				  strerror(ret));
			goto FAIL;
		}
		for (uint8_t j = 0; j < 3; j++) {
			selftest_response[j] +=
				static_cast<int32_t>(raw_gyro[j]);
		}
		sleep_ms(SELF_TEST_SLEEP);
	}

	ret = disableGyroSelfTest();
	if (ret != 0) {
		log_error("Error in disableGyroSelfTest(). (%s)\n",
			  strerror(ret));
		goto FAIL;
	}
	log_debug("     - Disabled\n");
	for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
		ret = getRawGyro(raw_gyro);
		if (ret != 0) {
			log_error("Error in getRawGyro(). (%s)\n",
				  strerror(ret));
			goto FAIL;
		}
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

	if (abs(m_gyro_self_test[0]) < SELF_TEST_GYRO_THRESHOLD &&
	    abs(m_gyro_self_test[1]) < SELF_TEST_GYRO_THRESHOLD &&
	    abs(m_gyro_self_test[2]) < SELF_TEST_GYRO_THRESHOLD) {
		log_info("   - [%0.3f, %0.3f, %0.3f] => PASSED\n",
			 m_gyro_self_test[0], m_gyro_self_test[1],
			 m_gyro_self_test[2]);
		return 0;
	}

	log_warning("   - [%0.3f, %0.3f, %0.3f] => FAILED\n",
		    m_gyro_self_test[0], m_gyro_self_test[1],
		    m_gyro_self_test[2]);

FAIL:
	m_self_test_fail = true;
	return ret;
}

int MPU6050::enableAccSelfTest()
{
	int ret = 0;

	ret = setAccRange(ACC_RANGE_8G);
	if (ret != 0) {
		log_error("Error in setAccRange(). (%s)\n", strerror(ret));
		return ret;
	}

	uint8_t buf;
	ret = read(ACC_CONF_ADDR, &buf, 1);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	buf |= X_ACC_SELF_TEST_BIT;
	buf |= Y_ACC_SELF_TEST_BIT;
	buf |= Z_ACC_SELF_TEST_BIT;
	uint8_t data[2] = { ACC_CONF_ADDR, buf };
	ret = i2c_write_blocking(i2c_default, m_address, data, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::disableAccSelfTest()
{
	int ret = 0;
	uint8_t buf;
	ret = read(ACC_CONF_ADDR, &buf, 1);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	buf &= 0b00011111; // reset self test bits
	uint8_t data[2] = { ACC_CONF_ADDR, buf };
	ret = i2c_write_blocking(i2c_default, m_address, data, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::enableGyroSelfTest()
{
	int ret = 0;

	ret = setGyroRange(GYRO_RANGE_250);
	if (ret != 0) {
		log_error("Error in setGyroRange(). (%s)\n", strerror(ret));
		return ret;
	}

	uint8_t buf;
	ret = read(GYRO_CONF_ADDR, &buf, 1);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	buf |= X_GYRO_SELF_TEST_BIT;
	buf |= Y_GYRO_SELF_TEST_BIT;
	buf |= Z_GYRO_SELF_TEST_BIT;
	uint8_t data[2] = { GYRO_CONF_ADDR, buf };
	ret = i2c_write_blocking(i2c_default, m_address, data, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::disableGyroSelfTest()
{
	int ret = 0;
	uint8_t buf;
	ret = read(GYRO_CONF_ADDR, &buf, 1);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	buf &= 0b00011111; // reset self test bits
	uint8_t data[2] = { GYRO_CONF_ADDR, buf };
	ret = i2c_write_blocking(i2c_default, m_address, data, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::getAcc(float accel[3])
{
	if (accel == nullptr) {
		log_error("Invalid argument\n");
		return EINVAL;
	}

	uint8_t buf[6];
	int ret = read(ACC_X_H_ADDR, buf, 6);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	accel[0] = static_cast<int16_t>((buf[0] << 8) | buf[1]) * m_acc_scale;
	accel[1] = static_cast<int16_t>((buf[2] << 8) | buf[3]) * m_acc_scale;
	accel[2] = static_cast<int16_t>((buf[4] << 8) | buf[5]) * m_acc_scale;

	return 0;
}

void MPU6050::printAcc(float accel[3])
{
	if (accel == nullptr) {
		log_error("Invalid argument\n");
		return;
	}

	printf("Acc = [%f, %f, %f]\t|.| = %f\n", accel[0], accel[1], accel[2],
	       arc::common::modf(accel));
}

int MPU6050::getRawAcc(int16_t accel[3])
{
	if (accel == nullptr) {
		log_error("Invalid argument\n");
		return EINVAL;
	}

	uint8_t buf[6];
	int ret = read(ACC_X_H_ADDR, buf, 6);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	accel[0] = (buf[0] << 8) | buf[1];
	accel[1] = (buf[2] << 8) | buf[3];
	accel[2] = (buf[4] << 8) | buf[5];

	return 0;
}

void MPU6050::printRawAcc(int16_t accel[3])
{
	if (accel == nullptr) {
		log_error("Invalid argument\n");
		return;
	}

	printf("Raw acc = [%d, %d, %d]\n", accel[0], accel[1], accel[2]);
}

int MPU6050::getGyro(float gyro[3])
{
	if (gyro == nullptr) {
		log_error("Invalid argument\n");
		return EINVAL;
	}

	uint8_t buf[6];
	int ret = read(GYRO_X_H_ADDR, buf, 6);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	gyro[0] = static_cast<int16_t>((buf[0] << 8) | buf[1]) * m_gyro_scale;
	gyro[1] = static_cast<int16_t>((buf[2] << 8) | buf[3]) * m_gyro_scale;
	gyro[2] = static_cast<int16_t>((buf[4] << 8) | buf[5]) * m_gyro_scale;

	return 0;
}

void MPU6050::printGyro(float gyro[3])
{
	if (gyro == nullptr) {
		log_error("Invalid argument\n");
		return;
	}

	printf("Gyro = [%f, %f, %f]\n", gyro[0], gyro[1], gyro[2]);
}

int MPU6050::getRawGyro(int16_t gyro[3])
{
	if (gyro == nullptr) {
		log_error("Invalid argument\n");
		return EINVAL;
	}

	uint8_t buf[6];
	int ret = read(GYRO_X_H_ADDR, buf, 6);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	gyro[0] = (buf[0] << 8) | buf[1];
	gyro[1] = (buf[2] << 8) | buf[3];
	gyro[2] = (buf[4] << 8) | buf[5];

	return 0;
}

void MPU6050::printRawGyro(int16_t gyro[3])
{
	if (gyro == nullptr) {
		log_error("Invalid argument\n");
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
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}
	sleep_ms(10);

	uint8_t buf2[] = { INT_ENABLE_ADDR, DATA_READY_INT_BIT };
	ret = i2c_write_blocking(i2c_default, m_address, buf2, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}
	sleep_ms(10);

	return 0;
}

int MPU6050::getInterruptStatus(uint8_t &int_status)
{
	int ret = read(INT_STATUS_ADDR, &int_status, 1);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	return 0;
}

int MPU6050::reset()
{
	int ret = 0;

	uint8_t pwr_mgmt;
	ret = read(PWR_MGMT_1_ADDR, &pwr_mgmt);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	uint8_t reset_mask = pwr_mgmt | RESET_BIT;
	uint8_t buf[] = { PWR_MGMT_1_ADDR, reset_mask };
	ret = i2c_write_blocking(i2c_default, m_address, buf, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}
	sleep_ms(100);

	return 0;
}

int MPU6050::reset_paths()
{
	uint8_t reset_mask = GYRO_RESET_BIT | ACC_RESET_BIT | TEMP_RESET_BIT;
	uint8_t buf[] = { SIGNAL_PATH_RESET_ADDR, reset_mask };
	int ret = i2c_write_blocking(i2c_default, m_address, buf, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}
	sleep_ms(100);

	return 0;
}

int MPU6050::sleep()
{
	int ret = 0;
	uint8_t pwr_mgmt;
	ret = read(PWR_MGMT_1_ADDR, &pwr_mgmt);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	uint8_t sleep_mask = pwr_mgmt & SLEEP_BIT;
	uint8_t buf2[] = { PWR_MGMT_1_ADDR, sleep_mask };
	ret = i2c_write_blocking(i2c_default, m_address, buf2, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}
	sleep_ms(10);

	return 0;
}

int MPU6050::wake()
{
	int ret = 0;
	uint8_t pwr_mgmt;
	ret = read(PWR_MGMT_1_ADDR, &pwr_mgmt);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	uint8_t wake_mask = pwr_mgmt & ~SLEEP_BIT;
	uint8_t buf2[] = { PWR_MGMT_1_ADDR, wake_mask };
	ret = i2c_write_blocking(i2c_default, m_address, buf2, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}
	sleep_ms(10);

	return 0;
}

int MPU6050::getDLPFConfig(dlpf_t &cfg)
{
	uint8_t dlpf_cfg;
	int ret = read(CONFIG_ADDR, &dlpf_cfg);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	cfg = static_cast<dlpf_t>(dlpf_cfg & DLPF_CFG_BITS);

	return 0;
}

int MPU6050::setDLPFConfig(dlpf_t cfg)
{
	int ret = 0;
	uint8_t dlpf_cfg;
	ret = read(CONFIG_ADDR, &dlpf_cfg);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	dlpf_cfg &= ~DLPF_CFG_BITS; // clear the dlpf bits
	dlpf_cfg |= cfg; // set the dlpf bits
	uint8_t buf2[] = { CONFIG_ADDR, dlpf_cfg };
	ret = i2c_write_blocking(i2c_default, m_address, buf2, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}
	sleep_ms(10);

	return 0;
}

} // MPU6050
} // namespace sensors
} // namespace arc
