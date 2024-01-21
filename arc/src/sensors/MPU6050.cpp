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
#include "common/common.hpp"
#include "common/log.hpp"
#include <errno.h>

namespace arc {
namespace sensors {

static constexpr uint8_t TEST_X_ADDR = 0x0D;
static constexpr uint8_t XA_TEST_4_2_BITS = 0xE0;
static constexpr uint8_t XG_TEST_BITS = 0x1F;

static constexpr uint8_t TEST_Y_ADDR = 0x0E;
static constexpr uint8_t YA_TEST_4_2_BITS = 0xE0;
static constexpr uint8_t YG_TEST_BITS = 0x1F;

static constexpr uint8_t TEST_Z_ADDR = 0x0F;
static constexpr uint8_t ZA_TEST_4_2_BITS = 0xE0;
static constexpr uint8_t ZG_TEST_BITS = 0x1F;

static constexpr uint8_t TEST_A_ADDR = 0x10;
static constexpr uint8_t XA_TEST_1_0_BITS = 0x30;
static constexpr uint8_t YA_TEST_1_0_BITS = 0x0C;
static constexpr uint8_t ZA_TEST_1_0_BITS = 0x02;

static constexpr uint8_t SMPLRT_DIV_ADDR = 0x19;

static constexpr uint8_t CONFIG_ADDR = 0x1A;
static constexpr uint8_t EXT_SYNC_SET_BITS = 0x38;
static constexpr uint8_t DLPF_CFG_BITS = 0x7;

static constexpr uint8_t GYRO_CONF_ADDR = 0x1B;
static constexpr uint8_t X_GYRO_SELF_TEST_BIT = (1 << 7);
static constexpr uint8_t Y_GYRO_SELF_TEST_BIT = (1 << 6);
static constexpr uint8_t Z_GYRO_SELF_TEST_BIT = (1 << 5);
static constexpr uint8_t GYRO_RANGE_BITS = 0x18;

static constexpr uint8_t ACC_CONF_ADDR = 0x1C;
static constexpr uint8_t X_ACC_SELF_TEST_BIT = (1 << 7);
static constexpr uint8_t Y_ACC_SELF_TEST_BIT = (1 << 6);
static constexpr uint8_t Z_ACC_SELF_TEST_BIT = (1 << 5);
static constexpr uint8_t ACC_SCALE_BITS = 0x18;
static constexpr uint8_t ACC_DHPF_BITS = 0x7;

static constexpr uint8_t FIFO_EN_ADDR = 0x23;
static constexpr uint8_t TEMP_FIFO_ENABLE_BIT = (1 << 7);
static constexpr uint8_t X_GYRO_FIFO_ENABLE_BIT = (1 << 6);
static constexpr uint8_t Y_GYRO_FIFO_ENABLE_BIT = (1 << 5);
static constexpr uint8_t Z_GYRO_FIFO_ENABLE_BIT = (1 << 4);
static constexpr uint8_t ACC_FIFO_ENABLE_BIT = (1 << 3);
static constexpr uint8_t SLV2_FIFO_ENABLE_BIT = (1 << 2);
static constexpr uint8_t SLV1_FIFO_ENABLE_BIT = (1 << 1);
static constexpr uint8_t SLV0_FIFO_ENABLE_BIT = (1 << 0);

static constexpr uint8_t INT_PIN_CFG_ADDR = 0x37;
/**
 * @brief When this bit is equal to 0, the logic level for the INT pin is active high. 
 * When this bit is equal to 1, the logic level for the INT pin is active low. 
 */
static constexpr uint8_t INT_LEVEL_BIT = (1 << 7);
/**
 * @brief When this bit is equal to 0, the INT pin is configured as push-pull.
 * When this bit is equal to 1, the INT pin is configured as open drain.
 */
static constexpr uint8_t INT_OPEN_BIT = (1 << 6);
/**
 * @brief When this bit is equal to 0, the INT pin emits a 50us long pulse.
 * When this bit is equal to 1, the INT pin is held high until the interrupt is cleared
 */
static constexpr uint8_t LATCH_INT_ENABLE_BIT = (1 << 5);
/**
 * @brief When this bit is equal to 0, interrupt status bits are cleared only by reading 
 * INT_STATUS (Register 58).
 * When this bit is equal to 1, interrupt status bits are cleared on any read operation.
 */
static constexpr uint8_t INT_CLEAR_ON_READ_BIT = (1 << 4);
static constexpr uint8_t FSYNC_INT_LEVEL_BIT = (1 << 3);
static constexpr uint8_t FSYNC_INT_ENABLE_BIT = (1 << 2);
static constexpr uint8_t I2C_BYPASS_ENABLE_BIT = (1 << 1);

static constexpr uint8_t INT_ENABLE_ADDR = 0x38;
static constexpr uint8_t FIFO_OVERFLOW_BIT = (1 << 4);
static constexpr uint8_t I2C_MASTER_INT_BIT = (1 << 3);
/**
 * @brief When set to 1, this bit enables the Data Ready interrupt, which occurs each 
 * time a write operation to all of the sensor registers has been completed.
 */
static constexpr uint8_t DATA_READY_INT_BIT = (1 << 0);

static constexpr uint8_t INT_STATUS_ADDR = 0x3A;
static constexpr uint8_t FIFO_OVERFLOW_INT_ENABLE_BIT = (1 << 4);
static constexpr uint8_t I2C_MASTER_INT_ENABLE_BIT = (1 << 3);
/**
 * @brief This bit automatically sets to 1 when a Data Ready interrupt is generated. 
 * The bit clears to 0 after the register has been read.
 */
static constexpr uint8_t DATA_READY_INT_ENABLE_BIT = (1 << 0);

static constexpr uint8_t ACC_X_H_ADDR = 0x3B;
static constexpr uint8_t ACC_X_L_ADDR = 0x3C;
static constexpr uint8_t ACC_Y_H_ADDR = 0x3D;
static constexpr uint8_t ACC_Y_L_ADDR = 0x3E;
static constexpr uint8_t ACC_Z_H_ADDR = 0x3F;
static constexpr uint8_t ACC_Z_L_ADDR = 0x40;

static constexpr uint8_t TEMP_H_ADDR = 0x41;
static constexpr uint8_t TEMP_L_ADDR = 0x42;

static constexpr uint8_t GYRO_X_H_ADDR = 0x43;
static constexpr uint8_t GYRO_X_L_ADDR = 0x44;
static constexpr uint8_t GYRO_Y_H_ADDR = 0x45;
static constexpr uint8_t GYRO_Y_L_ADDR = 0x46;
static constexpr uint8_t GYRO_Z_H_ADDR = 0x47;
static constexpr uint8_t GYRO_Z_L_ADDR = 0x48;

static constexpr uint8_t SIGNAL_PATH_RESET_ADDR = 0x68;
static constexpr uint8_t GYRO_RESET_BIT = (1 << 2);
static constexpr uint8_t ACC_RESET_BIT = (1 << 1);
static constexpr uint8_t TEMP_RESET_BIT = (1 << 0);

static constexpr uint8_t USER_CTRL_ADDR = 0x6A;
static constexpr uint8_t FIFO_ENABLED_BIT = (1 << 6);
static constexpr uint8_t I2C_MASTER_ENABLED_BIT = (1 << 5);
static constexpr uint8_t I2C_ENABLED_BIT = (1 << 4);
static constexpr uint8_t FIFO_RESET_BIT = (1 << 2);
static constexpr uint8_t I2C_MASTER_RESET_BIT = (1 << 1);
static constexpr uint8_t SIGNAL_COND_RESET_BIT = (1 << 0);

static constexpr uint8_t PWR_MGMT_1_ADDR = 0x6B;
static constexpr uint8_t RESET_BIT = (1 << 7);
static constexpr uint8_t SLEEP_BIT = (1 << 6);
static constexpr uint8_t CYCLE_BIT = (1 << 5);
static constexpr uint8_t TEMP_DISABLED_BIT = (1 << 3);
static constexpr uint8_t CLK_SEL_BITS = 0x7;

static constexpr uint8_t PWR_MGMT_2_ADDR = 0x6C;
static constexpr uint8_t LP_WAKE_CTRL_BITS = 0xC0;
static constexpr uint8_t STBY_XA_BIT = (1 << 5);
static constexpr uint8_t STBY_YA_BIT = (1 << 4);
static constexpr uint8_t STBY_ZA_BIT = (1 << 3);
static constexpr uint8_t STBY_XG_BIT = (1 << 2);
static constexpr uint8_t STBY_YG_BIT = (1 << 1);
static constexpr uint8_t STBY_ZG_BIT = (1 << 0);

static constexpr uint8_t FIFO_COUNT_H_ADDR = 0x72;

static constexpr uint8_t FIFO_COUNT_L_ADDR = 0x73;

static constexpr uint8_t FIFO_DATA_ADDR = 0x74;

static constexpr uint8_t WHO_AM_I_ADDR = 0x75;
static constexpr uint8_t WHO_AM_I_BITS = 0x7E;

static constexpr uint8_t SELF_TEST_SAMPLES = MPU6050_SELF_TEST_SAMPLES;
static constexpr uint8_t SELF_TEST_SLEEP = MPU6050_SELF_TEST_SLEEP;
static constexpr float SELF_TEST_ACC_THR = MPU6050_SELF_TEST_ACC_THR;
static constexpr float SELF_TEST_GYRO_THR = MPU6050_SELF_TEST_GYRO_THR;

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

MPU6050::MPU6050(i2c_inst_t *i2c_inst, uint8_t address)
	: m_i2c_inst{ i2c_inst }
	, m_addr{ address }
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

	ret = i2c_write_blocking(m_i2c_inst, m_addr, &reg, 1, true);
	if (ret != 1) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	ret = i2c_read_blocking(m_i2c_inst, m_addr, buf, bytes, false);
	if (ret != static_cast<int>(bytes)) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	return 0;
}

int MPU6050::setAccRange(AccRange range)
{
	if (range < AccRange::_2G || range > AccRange::_16G) {
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
	acc_conf |= static_cast<uint8_t>(range) << 3; // set the range bits
	uint8_t data[2] = { ACC_CONF_ADDR, acc_conf };
	ret = i2c_write_blocking(m_i2c_inst, m_addr, data, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	switch (range) {
	case AccRange::_2G:
		m_acc_scale = 1.0f / (1 << 14);
		log_debug("Acceleration scale set to 2g: %f\n", m_acc_scale);
		break;
	case AccRange::_4G:
		m_acc_scale = 1.0f / (1 << 13);
		log_debug("Acceleration scale set to 4g: %f\n", m_acc_scale);
		break;
	case AccRange::_8G:
		m_acc_scale = 1.0f / (1 << 12);
		log_debug("Acceleration scale set to 8g: %f\n", m_acc_scale);
		break;
	case AccRange::_16G:
		m_acc_scale = 1.0f / (1 << 11);
		log_debug("Acceleration scale set to 16g: %f\n", m_acc_scale);
		break;
	}

	sleep_ms(10);

	return 0;
}

int MPU6050::setGyroRange(GyroRange range)
{
	if (range < GyroRange::_250 || range > GyroRange::_2000) {
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
	gyro_conf |= static_cast<uint8_t>(range) << 3; // set the range bits
	uint8_t data[2] = { GYRO_CONF_ADDR, gyro_conf };
	ret = i2c_write_blocking(m_i2c_inst, m_addr, data, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}

	switch (range) {
	case GyroRange::_250:
		m_gyro_scale = 1.0f / (1 << 14);
		log_debug("Gyroscope scale set to 250°: %f\n", m_gyro_scale);
		break;
	case GyroRange::_500:
		m_gyro_scale = 1.0f / (1 << 13);
		log_debug("Gyroscope scale set to 500°: %f\n", m_gyro_scale);
		break;
	case GyroRange::_1000:
		m_gyro_scale = 1.0f / (1 << 12);
		log_debug("Gyroscope scale set to 1000°: %f\n", m_gyro_scale);
		break;
	case GyroRange::_2000:
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

	if (abs(m_acc_self_test[0]) < SELF_TEST_ACC_THR &&
	    abs(m_acc_self_test[1]) < SELF_TEST_ACC_THR &&
	    abs(m_acc_self_test[2]) < SELF_TEST_ACC_THR) {
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

	if (abs(m_gyro_self_test[0]) < SELF_TEST_GYRO_THR &&
	    abs(m_gyro_self_test[1]) < SELF_TEST_GYRO_THR &&
	    abs(m_gyro_self_test[2]) < SELF_TEST_GYRO_THR) {
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

	ret = setAccRange(AccRange::_8G);
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
	ret = i2c_write_blocking(m_i2c_inst, m_addr, data, 2, false);
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
	ret = i2c_write_blocking(m_i2c_inst, m_addr, data, 2, false);
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

	ret = setGyroRange(GyroRange::_250);
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
	ret = i2c_write_blocking(m_i2c_inst, m_addr, data, 2, false);
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
	ret = i2c_write_blocking(m_i2c_inst, m_addr, data, 2, false);
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
	ret = i2c_write_blocking(m_i2c_inst, m_addr, buf, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}
	sleep_ms(10);

	uint8_t buf2[] = { INT_ENABLE_ADDR, DATA_READY_INT_BIT };
	ret = i2c_write_blocking(m_i2c_inst, m_addr, buf2, 2, false);
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
	ret = i2c_write_blocking(m_i2c_inst, m_addr, buf, 2, false);
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
	int ret = i2c_write_blocking(m_i2c_inst, m_addr, buf, 2, false);
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
	ret = i2c_write_blocking(m_i2c_inst, m_addr, buf2, 2, false);
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
	ret = i2c_write_blocking(m_i2c_inst, m_addr, buf2, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}
	sleep_ms(10);

	return 0;
}

int MPU6050::getDLPFConfig(DlpfBW &cfg)
{
	uint8_t dlpf_cfg;
	int ret = read(CONFIG_ADDR, &dlpf_cfg);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	cfg = static_cast<DlpfBW>(dlpf_cfg & DLPF_CFG_BITS);

	return 0;
}

int MPU6050::setDLPFConfig(DlpfBW cfg)
{
	int ret = 0;
	uint8_t dlpf_cfg;
	ret = read(CONFIG_ADDR, &dlpf_cfg);
	if (ret != 0) {
		log_error("Error in read(). (%s)\n", strerror(ret));
		return ret;
	}

	dlpf_cfg &= ~DLPF_CFG_BITS; // clear the dlpf bits
	dlpf_cfg |= static_cast<uint8_t>(cfg); // set the dlpf bits
	uint8_t buf2[] = { CONFIG_ADDR, dlpf_cfg };
	ret = i2c_write_blocking(m_i2c_inst, m_addr, buf2, 2, false);
	if (ret != 2) {
		log_error("Error in i2c_write_blocking()\n");
		return EIO;
	}
	sleep_ms(10);

	return 0;
}

} // namespace sensors
} // namespace arc
