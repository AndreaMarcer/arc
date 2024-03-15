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

/*****************************************************************************\
|                                   INCLUDES                                  |
\*****************************************************************************/
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "Eigen/Core"

/*****************************************************************************\
|                                    MACROS                                   |
\*****************************************************************************/
#define MPU6050_SELF_TEST_SAMPLES 20
#define MPU6050_SELF_TEST_SLEEP 10
#define MPU6050_SELF_TEST_ACC_THR 0.1
#define MPU6050_SELF_TEST_GYRO_THR 0.1

#define MPU6050_GYRO_CALIB_SAMPLES 1000
#define MPU6050_GYRO_CALIB_THR 500
#define MPU6050_GYRO_CALIB_SLEEP 1

namespace arc::sensors {

/*****************************************************************************\
|                                   MPU6050                                   |
\*****************************************************************************/
class MPU6050 {
public:
    /**
     * @brief Digital Low PAss Filter cut-off frequency
     *
     * +-------------+-------------------+
     * |Accelerometer|      Gyroscope    |
     * |  BW | Delay |  BW | Delay |  Fs |
     * |  Hz |   ms  |  Hz |   ms  | kHz |
     * +-----+-------+-----+-------+-----+
     * | 260 |  0.0  | 256 |  0.98 |   8 |
     * | 184 |  2.0  | 188 |  1.90 |   1 |
     * |  94 |  3.0  |  98 |  2.80 |   1 |
     * |  44 |  4.9  |  42 |  4.80 |   1 |
     * |  21 |  8.5  |  20 |  6.30 |   1 |
     * |  10 | 13.8  |  10 | 13.40 |   1 |
     * |   5 | 19.0  |   5 | 18.60 |   1 |
     * +-----+-------+-----+-------+-----+
     *
     */
    enum class DlpfBW {
        _260Hz = 0,
        _184Hz = 1,
        _94Hz = 2,
        _44Hz = 3,
        _21Hz = 4,
        _10Hz = 5,
        _5Hz = 6,
    };

    /**
     * @brief Gyroscope full scale range [°/s].
     * 
     */
    enum class GyroRange {
        _250 = 0, //<! 4.36332 rad/s
        _500 = 1, //<! 8.72665 rad/s
        _1000 = 2, //<! 17.4533 rad/s
        _2000 = 3, //<! 34.90659 rad/s
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

    static constexpr uint8_t I2C_ADDR_AD0_LOW = 0x68;
    static constexpr uint8_t I2C_ADDR_AD0_HIGH = 0x69;

    MPU6050(i2c_inst_t *i2c_inst, uint8_t address);

    int read(uint8_t reg, uint8_t *buf, size_t bytes = 1);

    int getRawAcc(Eigen::Vector<int16_t, 3> &);
    int getAcc(Eigen::Vector<float, 3> &);
    int setAccRange(AccRange range);
    int enableAccSelfTest();
    int disableAccSelfTest();

    int getRawGyro(Eigen::Vector<int16_t, 3> &);
    int getGyro(Eigen::Vector<float, 3> &);
    int setGyroRange(GyroRange, bool = true);
    int getGyroRange(GyroRange &);
    int enableGyroSelfTest();
    int disableGyroSelfTest();
    void setGyroRad();
    void setGyroDeg();
    int calibrateGyro();

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
    i2c_inst_t *m_i2c_inst{nullptr};
    uint8_t m_addr{0};
    bool m_I2C_master_mode{false};
    bool m_I2C_enabled{false};

    float m_acc_scale{1.0};

    float m_gyro_scale{1.0};
    Eigen::Vector<int16_t, 3> m_gyro_offset{};
    bool m_gyro_in_rad{false};
    bool m_gyro_calibrated{false};

    bool m_self_test_fail{false};
};

}  // namespace arc::sensors
