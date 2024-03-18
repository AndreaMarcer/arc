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

/*****************************************************************************\
|                                   INCLUDES                                  |
\*****************************************************************************/
#include <errno.h>

#include "math.h"

#include "common/log.hpp"
#include "common/common.hpp"
#include "common/constants.hpp"
#include "sensors/MPU6050.hpp"

/*****************************************************************************\
|                                    MACROS                                   |
\*****************************************************************************/
#define MPU6050_GET_FUNC(FUNC_NAME, TYPE, ADDR, MASK, OFFSET)               \
    int MPU6050::get##FUNC_NAME(TYPE &value) {                              \
        int ret = 0;                                                        \
        uint8_t read_value;                                                 \
        ret = m_i2c.readBits(ADDR, MASK, OFFSET, read_value);               \
        if (ret != 1) {                                                     \
            log_error << "Error in readBits().\n";                          \
            return EIO;                                                     \
        }                                                                   \
        value = static_cast<TYPE>(read_value);                              \
        log_debug << "get" #FUNC_NAME ": " << static_cast<uint>(read_value) \
                  << std::endl;                                             \
        return 0;                                                           \
    }

#define MPU6050_SET_FUNC(FUNC_NAME, TYPE, ADDR, MASK, OFFSET)                 \
    int MPU6050::set##FUNC_NAME(TYPE value) {                                 \
        int ret = 0;                                                          \
        log_debug << "set" #FUNC_NAME ": " << static_cast<uint>(value)        \
                  << std::endl;                                               \
        ret =                                                                 \
            m_i2c.writeBits(ADDR, static_cast<uint8_t>(value), MASK, OFFSET); \
        if (ret != 1) {                                                       \
            log_error << "Error in writeBits().\n";                           \
            return EIO;                                                       \
        }                                                                     \
        TYPE written_value;                                                   \
        ret = get##FUNC_NAME(written_value);                                  \
        if (ret != 0) {                                                       \
            log_error << "Error in get" #FUNC_NAME "().\n";                   \
            return ret;                                                       \
        }                                                                     \
        if (written_value != value) {                                         \
            log_warning                                                       \
                << "Failed to write, make sure the chip is not sleeping.\n";  \
            return EIO;                                                       \
        }                                                                     \
        return 0;                                                             \
    }

namespace arc::sensors {

static float gyroFactoryTrim(uint8_t factory_trim, bool y) {
    if (factory_trim == 0) return 0.0f;
    return (y ? -1.0f : 1.0f) * 25.0f * 131.0f *
           powf(1.046f, factory_trim - 1.0f);
}

static float accFactoryTrim(uint8_t factory_trim) {
    if (factory_trim == 0) return 0.0f;
    float exp = (factory_trim - 1.0f) / ((1 << 5) - 2.0f);
    return 4096.0f * 0.34f * powf(0.92f / 0.34f, exp);
}

MPU6050::MPU6050(i2c_inst_t *i2c_inst, uint8_t address)
    : m_i2c{address, i2c_inst} {
    // TODO: add function return check
    int ret = 0;

    ret = reset();
    if (ret != 0) {
        log_error << "Error in reset(). (" << strerror(ret) << ")\n";
    }

    ret = reset_paths();
    if (ret != 0) {
        log_error << "Error in reset_paths(). (" << strerror(ret) << ")\n";
    }

    wake();

    ret = setClockSource(ClockSource::PLL_GYROX);
    if (ret != 0) {
        log_error << "Error in setClockSource(). (" << strerror(ret) << ")\n";
    }

    ret = setAccRange(AccRange::_8G);
    if (ret != 0) {
        log_error << "Error in setAccRange(). (" << strerror(ret) << ")\n";
    }

    ret = setGyroRange(GyroRange::_250);
    if (ret != 0) {
        log_error << "Error in setGyroRange(). (" << strerror(ret) << ")\n";
    }

    selfTest();
    if (!m_self_test_fail) {
        calibrateGyro();
    }
    sleep();
}

MPU6050_GET_FUNC(AccRange, AccRange, ACC_CONF_ADDR, ACC_RANGE_BITS,
                 ACC_RANGE_OFFSET)

int MPU6050::setAccRange(AccRange range) {
    int ret = 0;

    log_debug << "setAccRange: " << static_cast<uint>(range) << std::endl;

    ret = m_i2c.writeBits(ACC_CONF_ADDR, static_cast<uint8_t>(range),
                          ACC_RANGE_BITS, ACC_RANGE_OFFSET);
    if (ret != 1) {
        log_error << "Error in writeBits().\n";
        return EIO;
    }

    AccRange written_range;
    ret = getAccRange(written_range);
    if (ret != 0) {
        log_error << "Error in getAccRange().\n";
        return ret;
    }
    if (written_range != range) {
        log_warning << "Failed to write the accelerometer range, make sure the "
                       "chip is not sleeping.\n";
        return EIO;
    }

    switch (range) {
        case AccRange::_2G:
            m_acc_scale = 1.0f / (1 << 14);
            log_debug << "Acceleration scale set to 2g: " << m_acc_scale
                      << "\n";
            break;
        case AccRange::_4G:
            m_acc_scale = 1.0f / (1 << 13);
            log_debug << "Acceleration scale set to 4g: " << m_acc_scale
                      << "\n";
            break;
        case AccRange::_8G:
            m_acc_scale = 1.0f / (1 << 12);
            log_debug << "Acceleration scale set to 8g: " << m_acc_scale
                      << "\n";
            break;
        case AccRange::_16G:
            m_acc_scale = 1.0f / (1 << 11);
            log_debug << "Acceleration scale set to 16g: " << m_acc_scale
                      << "\n";
            break;
    }

    sleep_ms(10);

    return 0;
}

MPU6050_GET_FUNC(GyroRange, GyroRange, GYRO_CONF_ADDR, GYRO_RANGE_BITS,
                 GYRO_RANGE_OFFSET)

int MPU6050::setGyroRange(GyroRange range) {
    int ret = 0;

    log_debug << "setGyroRange: " << static_cast<uint>(range) << std::endl;

    ret = m_i2c.writeBits(GYRO_CONF_ADDR, static_cast<uint8_t>(range),
                          GYRO_RANGE_BITS, GYRO_RANGE_OFFSET);
    if (ret != 1) {
        log_error << "Error in writeBits().\n";
        return EIO;
    }

    GyroRange written_range;
    ret = getGyroRange(written_range);
    if (ret != 0) {
        log_error << "Error in getGyroRange().\n";
        return ret;
    }
    if (written_range != range) {
        log_warning << "Failed to write the gyroscope range, make sure the "
                       "chip is not sleeping.\n";
        return EIO;
    }

    switch (range) {
        case GyroRange::_250:
            m_gyro_scale = 1.0f / (1 << 14);
            log_debug << "Gyroscope scale set to 250°: " << m_gyro_scale
                      << "\n";
            break;
        case GyroRange::_500:
            m_gyro_scale = 1.0f / (1 << 13);
            log_debug << "Gyroscope scale set to 500°: " << m_gyro_scale
                      << "\n";
            break;
        case GyroRange::_1000:
            m_gyro_scale = 1.0f / (1 << 12);
            log_debug << "Gyroscope scale set to 1000°: " << m_gyro_scale
                      << "\n";
            break;
        case GyroRange::_2000:
            m_gyro_scale = 1.0f / (1 << 11);
            log_debug << "Gyroscope scale set to 2000°: " << m_gyro_scale
                      << "\n";
            break;
    }

    if (m_gyro_in_rad) {
        m_gyro_scale *= arc::common::DEG2RAD;
        log_debug << "Gyro scale set to radians: " << m_gyro_scale << "\n";
    }

    sleep_ms(10);

    return 0;
}

void MPU6050::setGyroRad() {
    if (!m_gyro_in_rad) {
        m_gyro_scale *= arc::common::DEG2RAD;
        m_gyro_in_rad = true;
        log_debug << "Gyro scale set to radians: " << m_gyro_scale << "\n";
    } else {
        log_debug << "Gyro already set to radians: " << m_gyro_scale << "\n";
    }
}

void MPU6050::setGyroDeg() {
    if (m_gyro_in_rad) {
        m_gyro_scale *= arc::common::RAD2DEG;
        m_gyro_in_rad = false;
        log_debug << "Gyro scale set to degrees: " << m_gyro_scale << "\n";
    } else {
        log_debug << "Gyro already set to degrees: " << m_gyro_scale << "\n";
    }
}

int MPU6050::selfTest() {
    log_info << "\n";
    log_info << "SELF TEST:\n";

    int ret = 0;

#ifdef LOG_DEBUG
    uint8_t buf[4];
    ret = m_i2c.readBytes(TEST_X_ADDR, buf, 4);
    if (ret != 4) {
        log_error << "Error in readBytes().\n";
        return EIO;
    }

    log_debug << " - REGISTERS:\n";
    for (uint8_t i = 0; i < 4; i++) {
        log_debug << "   - 0x" << HEX2STR(0x0D + i, 2) << ": "
                  << BYTE2STR(buf[i]) << std::endl;
    }
#endif

    ret = accSelfTest();
    if (ret != 0) {
        log_error << "Error in accSelfTest(). (" << strerror(ret) << ")\n";
        return ret;
    }
    if (m_self_test_fail) {
        log_critical << "Accelerometer Self Test failed\n";
        return 0;
    }

    ret = gyroSelfTest();
    if (ret != 0) {
        log_error << "Error in gyroSelfTest(). (" << strerror(ret) << ")\n";
        return ret;
    }
    if (m_self_test_fail) {
        log_critical << "Accelerometer Self Test failed\n";
    }

    return 0;
}

int MPU6050::accSelfTest() {
    log_info << " - Accelerometer:\n";

    uint8_t buf[4];
    float FT[3];
    int8_t acc_fact_test[3];
    uint8_t reg_10_mask = 0b00110000;
    Eigen::Vector<int16_t, 3> raw_acc;
    int32_t selftest_response[3] = {0};

    int ret = 0;

    ret = m_i2c.readBytes(TEST_X_ADDR, buf, 4);
    if (ret != 4) {
        log_error << "Error in readBytes().\n";
        goto FAIL;
    }

    log_debug << "   - FT: [";
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t acc_fact_test_4_2 = ((buf[i] & 0b11100000) >> 3);
        uint8_t acc_fact_test_1_0 = ((buf[3] & reg_10_mask) >> (4 - i * 2));
        acc_fact_test[i] = acc_fact_test_4_2 | acc_fact_test_1_0;

        FT[i] = accFactoryTrim(acc_fact_test[i]);
        log_debug_s << FT[i] << ", ";
        reg_10_mask >>= 2;
    }
    log_debug_s << "]\n";

    log_debug << "   - Self Test \n";
    ret = enableAccSelfTest();
    if (ret != 0) {
        log_error << "Error in enableAccSelfTest(). (" << strerror(ret)
                  << ")\n";
        goto FAIL;
    }
    log_debug << "     - Enabled\n";
    for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
        ret = getRawAcc(raw_acc);
        if (ret != 0) {
            log_error << "Error in getRawAcc(). (" << strerror(ret) << ")\n";
            goto FAIL;
        }
        for (uint8_t j = 0; j < 3; j++) {
            selftest_response[j] += static_cast<int32_t>(raw_acc[j]);
        }
        sleep_ms(SELF_TEST_SLEEP);
    }

    ret = disableAccSelfTest();
    if (ret != 0) {
        log_error << "Error in disableAccSelfTest(). (" << strerror(ret)
                  << ")\n";
        goto FAIL;
    }
    log_debug << "     - Disabled\n";
    for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
        ret = getRawAcc(raw_acc);
        if (ret != 0) {
            log_error << "Error in getRawAcc(). (" << strerror(ret) << ")\n";
            goto FAIL;
        }
        for (uint8_t j = 0; j < 3; j++) {
            selftest_response[j] -= static_cast<int32_t>(raw_acc[j]);
        }
        sleep_ms(SELF_TEST_SLEEP);
    }

    float acc_self_test[3];
    for (uint8_t i = 0; i < 3; i++) {
        acc_self_test[i] =
            ((selftest_response[i] / SELF_TEST_SAMPLES) - FT[i]) / FT[i];
    }

    if (abs(acc_self_test[0]) < SELF_TEST_ACC_THR &&
        abs(acc_self_test[1]) < SELF_TEST_ACC_THR &&
        abs(acc_self_test[2]) < SELF_TEST_ACC_THR) {
        log_info << "   - [" << std::setprecision(3) << acc_self_test[0] << ", "
                 << acc_self_test[1] << ", " << acc_self_test[2]
                 << "] => PASSED" << std::endl;
        return 0;
    }
    log_critical << "   - [" << std::setprecision(3) << acc_self_test[0] << ", "
                 << acc_self_test[1] << ", " << acc_self_test[2]
                 << "] => FAILED" << std::endl;

FAIL:
    m_self_test_fail = true;
    return ret;
}

int MPU6050::gyroSelfTest() {
    log_info << " - Gyroscope:\n";

    uint8_t buf[4];
    float FT[3];
    int8_t gyro_fact_test[3];
    Eigen::Vector<int16_t, 3> raw_gyro;
    int32_t selftest_response[3] = {0};

    int ret = 0;
    ret = m_i2c.readBytes(TEST_X_ADDR, buf, 4);
    if (ret != 4) {
        log_error << "Error in readBytes().\n";
        goto FAIL;
    }

    log_debug << "   - FT: [";
    for (uint8_t i = 0; i < 3; i++) {
        gyro_fact_test[i] = buf[i] & 0b00011111;
        FT[i] = gyroFactoryTrim(gyro_fact_test[i], i == 1);
        log_debug_s << FT[i] << ", ";
    }
    log_debug_s << "]\n";

    log_debug << "   - Self Test \n";
    ret = enableGyroSelfTest();
    if (ret != 0) {
        log_error << "Error in enableGyroSelfTest(). (" << strerror(ret)
                  << ")\n";
        goto FAIL;
    }
    log_debug << "     - Enabled\n";
    for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
        ret = getRawGyro(raw_gyro);
        if (ret != 0) {
            log_error << "Error in getRawGyro(). (" << strerror(ret) << ")\n";
            goto FAIL;
        }
        for (uint8_t j = 0; j < 3; j++) {
            selftest_response[j] += static_cast<int32_t>(raw_gyro[j]);
        }
        sleep_ms(SELF_TEST_SLEEP);
    }

    ret = disableGyroSelfTest();
    if (ret != 0) {
        log_error << "Error in disableGyroSelfTest(). (" << strerror(ret)
                  << ")\n";
        goto FAIL;
    }
    log_debug << "     - Disabled\n";
    for (uint8_t i = 0; i < SELF_TEST_SAMPLES; i++) {
        ret = getRawGyro(raw_gyro);
        if (ret != 0) {
            log_error << "Error in getRawGyro(). (" << strerror(ret) << ")\n";
            goto FAIL;
        }
        for (uint8_t j = 0; j < 3; j++) {
            selftest_response[j] -= static_cast<int32_t>(raw_gyro[j]);
        }
        sleep_ms(SELF_TEST_SLEEP);
    }

    float gyro_self_test[3];
    for (uint8_t i = 0; i < 3; i++) {
        gyro_self_test[i] =
            ((selftest_response[i] / SELF_TEST_SAMPLES) - FT[i]) / FT[i];
    }

    if (abs(gyro_self_test[0]) < SELF_TEST_GYRO_THR &&
        abs(gyro_self_test[1]) < SELF_TEST_GYRO_THR &&
        abs(gyro_self_test[2]) < SELF_TEST_GYRO_THR) {
        log_info << "   - [" << std::setprecision(3) << gyro_self_test[0]
                 << ", " << gyro_self_test[1] << ", " << gyro_self_test[2]
                 << "] => PASSED" << std::endl;
        return 0;
    }

    log_critical << "   - [" << std::setprecision(3) << gyro_self_test[0]
                 << ", " << gyro_self_test[1] << ", " << gyro_self_test[2]
                 << "] => FAILED" << std::endl;
FAIL:
    m_self_test_fail = true;
    return ret;
}

int MPU6050::enableAccSelfTest() {
    int ret = 0;

    ret = setAccRange(AccRange::_8G);
    if (ret != 0) {
        log_error << "Error in setAccRange(). (" << strerror(ret) << ")\n";
        return ret;
    }

    uint8_t mask =
        X_ACC_SELF_TEST_BIT | Y_ACC_SELF_TEST_BIT | Z_ACC_SELF_TEST_BIT;
    ret = m_i2c.setBits(ACC_CONF_ADDR, mask);
    if (ret != 1) {
        log_error << "Error in setBits().\n";
        return EIO;
    }

    sleep_ms(10);

    return 0;
}

int MPU6050::disableAccSelfTest() {
    int ret = 0;

    uint8_t mask =
        X_ACC_SELF_TEST_BIT | Y_ACC_SELF_TEST_BIT | Z_ACC_SELF_TEST_BIT;
    ret = m_i2c.unsetBits(ACC_CONF_ADDR, mask);
    if (ret != 1) {
        log_error << "Error in unsetBits().\n";
        return EIO;
    }

    sleep_ms(10);

    return 0;
}

int MPU6050::enableGyroSelfTest() {
    int ret = 0;

    ret = setGyroRange(GyroRange::_250);
    if (ret != 0) {
        log_error << "Error in setGyroRange(). (" << strerror(ret) << ")\n";
        return ret;
    }

    uint8_t mask =
        X_GYRO_SELF_TEST_BIT | Y_GYRO_SELF_TEST_BIT | Z_GYRO_SELF_TEST_BIT;
    ret = m_i2c.setBits(GYRO_CONF_ADDR, mask);
    if (ret != 1) {
        log_error << "Error in unsetBits().\n";
        return EIO;
    }

    sleep_ms(10);

    return 0;
}

int MPU6050::disableGyroSelfTest() {
    int ret = 0;

    uint8_t mask =
        X_GYRO_SELF_TEST_BIT | Y_GYRO_SELF_TEST_BIT | Z_GYRO_SELF_TEST_BIT;
    ret = m_i2c.unsetBits(GYRO_CONF_ADDR, mask);
    if (ret != 1) {
        log_error << "Error in unsetBits().\n";
        return EIO;
    }

    sleep_ms(10);

    return 0;
}

int MPU6050::enableInterrupt() {
    int ret;

    uint8_t buf[] = {INT_PIN_CFG_ADDR, LATCH_INT_ENABLE_BIT};
    ret = m_i2c.writeBytes(buf, 2);
    if (ret != 2) {
        log_error << "Error in writeBytes()\n";
        return EIO;
    }

    sleep_ms(10);

    uint8_t buf2[] = {INT_ENABLE_ADDR, DATA_READY_INT_BIT};
    ret = m_i2c.writeBytes(buf2, 2);
    if (ret != 2) {
        log_error << "Error in writeBytes()\n";
        return EIO;
    }
    sleep_ms(10);

    return 0;
}

int MPU6050::getInterruptStatus(uint8_t &int_status) {
    int ret = m_i2c.readBytes(INT_STATUS_ADDR, &int_status);
    if (ret != 1) {
        log_error << "Error in readBytes().\n";
        return EIO;
    }
    return 0;
}

int MPU6050::reset() {
    int ret = 0;

    ret = m_i2c.setBits(PWR_MGMT_1_ADDR, RESET_BIT);
    if (ret != 1) {
        log_error << "Error in setBits().\n";
        return EIO;
    }

    sleep_ms(100);

    return 0;
}

int MPU6050::reset_paths() {
    int ret = 0;

    uint8_t mask = GYRO_RESET_BIT | ACC_RESET_BIT | TEMP_RESET_BIT;
    ret = m_i2c.setBits(SIGNAL_PATH_RESET_ADDR, mask);
    if (ret != 1) {
        log_error << "Error in unsetBits().\n";
        return EIO;
    }

    sleep_ms(100);

    return 0;
}

int MPU6050::sleep() {
    int ret = 0;

    ret = m_i2c.setBits(PWR_MGMT_1_ADDR, SLEEP_BIT);
    if (ret != 1) {
        log_error << "Error in setBits()\n";
        return EIO;
    }
    sleep_ms(10);

    return 0;
}

int MPU6050::wake() {
    int ret = 0;

    ret = m_i2c.unsetBits(PWR_MGMT_1_ADDR, SLEEP_BIT);
    if (ret != 1) {
        log_error << "Error in setBits()\n";
        return EIO;
    }
    sleep_ms(10);

    return 0;
}

int MPU6050::calibrateGyro() {
    int ret = 0;
    m_gyro_calibrated = false;

    int16_t gyro_offset[3] = {0, 0, 0};
    setGyroOffset(gyro_offset);

    sleep_ms(100);

    GyroRange gyro_range;
    ret = getGyroRange(gyro_range);
    if (ret != 0) {
        log_error << "Error in getGyroRange(). (" << strerror(ret) << ")\n";
        return ret;
    }

    log_info << "Gyro Calibration. STAY STILL\n";
    Eigen::Vector<int16_t, 3> raw_gyro[GYRO_CALIB_SAMPLES];
    for (uint16_t i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        ret = getRawGyro(raw_gyro[i]);
        if (ret != 0) {
            log_error << "Error in getRawGyro(). (" << strerror(ret) << ")\n";
            return ret;
        }
        sleep_ms(GYRO_CALIB_SLEEP);
    }
    log_info << "Gyro Calibration FINISHED\n";

    int64_t gyro_sum[3] = {0};
    for (uint16_t i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            gyro_sum[j] += static_cast<int64_t>(raw_gyro[i][j]);
        }
    }

    log_debug << "Gyro mean: ";
    int16_t gyro_mean[3] = {0};
    for (uint8_t j = 0; j < 3; j++) {
        gyro_mean[j] = gyro_sum[j] / GYRO_CALIB_SAMPLES;
        log_debug_s << gyro_mean[j] << ", ";
    }
    log_debug_s << std::endl;

    uint64_t gyro_sum_squared[3] = {0};
    for (uint16_t i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            gyro_sum_squared[j] += (raw_gyro[i][j] - gyro_mean[j]) *
                                   (raw_gyro[i][j] - gyro_mean[j]);
        }
    }

    log_debug << "Gyro STD: ";
    uint64_t gyro_std[3] = {0};
    for (uint8_t j = 0; j < 3; j++) {
        gyro_std[j] = gyro_sum_squared[j] / (GYRO_CALIB_SAMPLES - 1);
        log_debug_s << gyro_std[j] << ", ";
    }
    log_debug_s << "\n";

    uint64_t gyro_std_thr =
        GYRO_CALIB_THR / (1 << ((static_cast<uint64_t>(gyro_range) * 2)));
    log_debug << "Gyro STD Threshold: " << gyro_std_thr << "\n";
    for (uint8_t j = 0; j < 3; j++) {
        if (gyro_std[j] > gyro_std_thr) {
            log_critical << "Gyroscope calibration FAILED. STD too high\n";
            return 0;
        }
    }

    log_debug << "Gyro Offset: ";
    for (uint8_t j = 0; j < 3; j++) {
        log_debug_s << -gyro_mean[j] << ", ";
        m_gyro_offset[j] = -gyro_mean[j];
        gyro_offset[j] = -gyro_mean[j];
    }
    log_debug_s << std::endl;

    setGyroOffset(gyro_offset);

    m_gyro_calibrated = true;
    return 0;
}

int MPU6050::setAccOffset(int16_t *offsets) {
    int ret = 0;

    uint16_t scaled_offsets[3];
    for (uint8_t i = 0; i < 3; i++) {
        scaled_offsets[i] = offsets[i];
    }

    ret = m_i2c.writeWords(XA_OFFS_H_ADDR, (uint16_t *)scaled_offsets, 3);
    if (ret != 3) {
        log_error << "Error in writeBytes()\n";
        return EIO;
    }
    return 0;
}

int MPU6050::setGyroOffset(int16_t *offsets) {
    int ret = 0;

    uint16_t scaled_offsets[3];
    for (uint8_t i = 0; i < 3; i++) {
        scaled_offsets[i] = offsets[i] / 4;
    }

    ret = m_i2c.writeWords(XG_OFFS_H_ADDR, (uint16_t *)scaled_offsets, 3);
    if (ret != 3) {
        log_error << "Error in writeBytes()\n";
        return EIO;
    }
    return 0;
}

int MPU6050::dumpMemory(uint8_t start) { return dumpMemory(start, start); }

int MPU6050::dumpMemory(uint8_t start, uint8_t end) {
    int ret = 0;
    uint8_t data;
    for (uint8_t i = start; i <= end; i++) {
        ret = m_i2c.readBytes(i, &data);
        if (ret != 1) {
            log_error << "Error in readBytes().\n";
            return EIO;
        }
        log_info << "0x" << HEX2STR((unsigned int)i, 2) << ": "
                 << BYTE2STR(data) << std::endl;
    }
    return 0;
}

MPU6050_GET_FUNC(DLPFConfig, DlpfBW, CONFIG_ADDR, DLPF_CFG_BITS,
                 DLPF_CFG_OFFSET)
MPU6050_SET_FUNC(DLPFConfig, DlpfBW, CONFIG_ADDR, DLPF_CFG_BITS,
                 DLPF_CFG_OFFSET)

MPU6050_GET_FUNC(ClockSource, ClockSource, PWR_MGMT_1_ADDR, CLK_SEL_BITS,
                 CLK_SEL_OFFSET)
MPU6050_SET_FUNC(ClockSource, ClockSource, PWR_MGMT_1_ADDR, CLK_SEL_BITS,
                 CLK_SEL_OFFSET)

}  // namespace arc::sensors
