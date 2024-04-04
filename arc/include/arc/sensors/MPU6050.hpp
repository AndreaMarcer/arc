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

#include "common/i2c.hpp"

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
    /************************************************************************\
    |                             CONSTEXPRS                                 |
    \************************************************************************/

    // clang-format off
    static constexpr uint8_t XG_OFFS_TC_ADDR    = 0x00;
    static constexpr uint8_t XG_PWR_MODE_BIT    = BIT(7);
    static constexpr uint8_t XG_OFFS_BITS       = 0b01111110;
    static constexpr uint8_t XG_OTP_BNK_VLD_BIT = BIT(0);

    static constexpr uint8_t YG_OFFS_TC_ADDR    = 0x01; 
    static constexpr uint8_t YG_PWR_MODE_BIT    = BIT(7);
    static constexpr uint8_t YG_OFFS_BITS       = 0b01111110;
    static constexpr uint8_t YG_OTP_BNK_VLD_BIT = BIT(0);

    static constexpr uint8_t ZG_OFFS_TC_ADDR    = 0x02; 
    static constexpr uint8_t ZG_PWR_MODE_BIT    = BIT(7);
    static constexpr uint8_t ZG_OFFS_BITS       = 0b01111110;
    static constexpr uint8_t ZG_OTP_BNK_VLD_BIT = BIT(0);

    static constexpr uint8_t X_FINE_GAIN_ADDR    = 0x03;
    static constexpr uint8_t XG_FINE_GAIN_BITS   = 0b00001111;
    static constexpr uint8_t XG_FINE_GAIN_OFFSET = 0;
    static constexpr uint8_t XA_FINE_GAIN_BITS   = 0b11110000;
    static constexpr uint8_t XA_FINE_GAIN_OFFSET = 4;

    static constexpr uint8_t Y_FINE_GAIN_ADDR    = 0x04;
    static constexpr uint8_t YG_FINE_GAIN_BITS   = 0b00001111;
    static constexpr uint8_t YG_FINE_GAIN_OFFSET = 0;
    static constexpr uint8_t YA_FINE_GAIN_BITS   = 0b11110000;
    static constexpr uint8_t YA_FINE_GAIN_OFFSET = 4;

    static constexpr uint8_t Z_FINE_GAIN_ADDR    = 0x05;
    static constexpr uint8_t ZG_FINE_GAIN_BITS   = 0b00001111;
    static constexpr uint8_t ZG_FINE_GAIN_OFFSET = 0;
    static constexpr uint8_t ZA_FINE_GAIN_BITS   = 0b11110000;
    static constexpr uint8_t ZA_FINE_GAIN_OFFSET = 4;

    static constexpr uint8_t XA_OFFS_H_ADDR = 0x06;
    static constexpr uint8_t XA_OFFS_L_ADDR = 0x07;
    static constexpr uint8_t YA_OFFS_H_ADDR = 0x08;
    static constexpr uint8_t YA_OFFS_L_ADDR = 0x09;
    static constexpr uint8_t ZA_OFFS_H_ADDR = 0x0A;
    static constexpr uint8_t ZA_OFFS_L_ADDR = 0x0B;

    static constexpr uint8_t TEST_X_ADDR      = 0x0D;
    static constexpr uint8_t XA_TEST_4_2_BITS = 0xE0;
    static constexpr uint8_t XG_TEST_BITS     = 0x1F;

    static constexpr uint8_t TEST_Y_ADDR      = 0x0E;
    static constexpr uint8_t YA_TEST_4_2_BITS = 0xE0;
    static constexpr uint8_t YG_TEST_BITS     = 0x1F;

    static constexpr uint8_t TEST_Z_ADDR      = 0x0F;
    static constexpr uint8_t ZA_TEST_4_2_BITS = 0xE0;
    static constexpr uint8_t ZG_TEST_BITS     = 0x1F;

    static constexpr uint8_t TEST_A_ADDR      = 0x10;
    static constexpr uint8_t XA_TEST_1_0_BITS = 0x30;
    static constexpr uint8_t YA_TEST_1_0_BITS = 0x0C;
    static constexpr uint8_t ZA_TEST_1_0_BITS = 0x02;

    static constexpr uint8_t XG_OFFS_H_ADDR = 0x13;
    static constexpr uint8_t XG_OFFS_L_ADDR = 0x14;
    static constexpr uint8_t YG_OFFS_H_ADDR = 0x15;
    static constexpr uint8_t YG_OFFS_L_ADDR = 0x16;
    static constexpr uint8_t ZG_OFFS_H_ADDR = 0x17;
    static constexpr uint8_t ZG_OFFS_L_ADDR = 0x18;

    static constexpr uint8_t SMPLRT_DIV_ADDR = 0x19;

    static constexpr uint8_t CONFIG_ADDR       = 0x1A;
    static constexpr uint8_t EXT_SYNC_SET_BITS = 0x38;
    static constexpr uint8_t DLPF_CFG_BITS     = 0b00000111;
    static constexpr uint8_t DLPF_CFG_OFFSET   = 0;

    static constexpr uint8_t GYRO_CONF_ADDR       = 0x1B;
    static constexpr uint8_t X_GYRO_SELF_TEST_BIT = BIT(7);
    static constexpr uint8_t Y_GYRO_SELF_TEST_BIT = BIT(6);
    static constexpr uint8_t Z_GYRO_SELF_TEST_BIT = BIT(5);
    static constexpr uint8_t GYRO_RANGE_BITS      = 0b00011000;
    static constexpr uint8_t GYRO_RANGE_OFFSET    = 3;

    static constexpr uint8_t ACC_CONF_ADDR       = 0x1C;
    static constexpr uint8_t X_ACC_SELF_TEST_BIT = BIT(7);
    static constexpr uint8_t Y_ACC_SELF_TEST_BIT = BIT(6);
    static constexpr uint8_t Z_ACC_SELF_TEST_BIT = BIT(5);
    static constexpr uint8_t ACC_RANGE_BITS      = 0b00011000;
    static constexpr uint8_t ACC_RANGE_OFFSET    = 3;
    static constexpr uint8_t ACC_DHPF_BITS       = 0b00000111;
    static constexpr uint8_t ACC_DHPF_OFFSET     = 0;

    static constexpr uint8_t FF_THR_ADDR    = 0x1D;
    static constexpr uint8_t FF_DUR_ADDR    = 0x1E;
    static constexpr uint8_t MOT_THR_ADDR   = 0x1F;
    static constexpr uint8_t MOT_DUR_ADDR   = 0x20;
    static constexpr uint8_t ZRMOT_THR_ADDR = 0x21;
    static constexpr uint8_t ZRMOT_DUR_ADDR = 0x22;

    static constexpr uint8_t FIFO_EN_ADDR           = 0x23;
    static constexpr uint8_t TEMP_FIFO_ENABLE_BIT   = BIT(7);
    static constexpr uint8_t X_GYRO_FIFO_ENABLE_BIT = BIT(6);
    static constexpr uint8_t Y_GYRO_FIFO_ENABLE_BIT = BIT(5);
    static constexpr uint8_t Z_GYRO_FIFO_ENABLE_BIT = BIT(4);
    static constexpr uint8_t ACC_FIFO_ENABLE_BIT    = BIT(3);
    static constexpr uint8_t SLV2_FIFO_ENABLE_BIT   = BIT(2);
    static constexpr uint8_t SLV1_FIFO_ENABLE_BIT   = BIT(1);
    static constexpr uint8_t SLV0_FIFO_ENABLE_BIT   = BIT(0);

    static constexpr uint8_t INT_PIN_CFG_ADDR = 0x37;
    /**
     * @brief When this bit is equal to 0, the logic level for the INT pin is
     * active high. When this bit is equal to 1, the logic level for the INT 
     * pin is active low.
     */
    static constexpr uint8_t INT_LEVEL_BIT = BIT(7);
    /**
     * @brief When this bit is equal to 0, the INT pin is configured as 
     * push-pull. When this bit is equal to 1, the INT pin is configured as 
     * open drain.
     */
    static constexpr uint8_t INT_OPEN_BIT = BIT(6);
    /**
     * @brief When this bit is equal to 0, the INT pin emits a 50us long 
     * pulse. When this bit is equal to 1, the INT pin is held high until the
     * interrupt is cleared.
     */
    static constexpr uint8_t LATCH_INT_ENABLE_BIT = BIT(5);
    /**
     * @brief When this bit is equal to 0, interrupt status bits are cleared 
     * only by reading INT_STATUS (Register 58). When this bit is equal to 1,
     * interrupt status bits are cleared on any read operation.
     */
    static constexpr uint8_t INT_CLEAR_ON_READ_BIT = BIT(4);
    static constexpr uint8_t FSYNC_INT_LEVEL_BIT   = BIT(3);
    static constexpr uint8_t FSYNC_INT_ENABLE_BIT  = BIT(2);
    static constexpr uint8_t I2C_BYPASS_ENABLE_BIT = BIT(1);

    static constexpr uint8_t INT_ENABLE_ADDR    = 0x38;
    static constexpr uint8_t FIFO_OVERFLOW_BIT  = BIT(4);
    static constexpr uint8_t I2C_MASTER_INT_BIT = BIT(3);
    /**
     * @brief When set to 1, this bit enables the Data Ready interrupt, which
     * occurs each time a write operation to all of the sensor registers has
     * been completed.
     */
    static constexpr uint8_t DATA_READY_INT_BIT = BIT(0);

    static constexpr uint8_t DMP_INT_STATUS_ADDR = 0x39;

    static constexpr uint8_t INT_STATUS_ADDR              = 0x3A;
    static constexpr uint8_t FIFO_OVERFLOW_INT_ENABLE_BIT = BIT(4);
    static constexpr uint8_t I2C_MASTER_INT_ENABLE_BIT    = BIT(3);
    /**
     * @brief This bit automatically sets to 1 when a Data Ready interrupt is
     * generated. The bit clears to 0 after the register has been read.
     */
    static constexpr uint8_t DATA_READY_INT_ENABLE_BIT = BIT(0);

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

    static constexpr uint8_t EXT_SENS_DATA_00_ADDR   = 0x49;
    static constexpr uint8_t EXT_SENS_DATA_01_ADDR   = 0x4A;
    static constexpr uint8_t EXT_SENS_DATA_02_ADDR   = 0x4B;
    static constexpr uint8_t EXT_SENS_DATA_03_ADDR   = 0x4C;
    static constexpr uint8_t EXT_SENS_DATA_04_ADDR   = 0x4D;
    static constexpr uint8_t EXT_SENS_DATA_05_ADDR   = 0x4E;
    static constexpr uint8_t EXT_SENS_DATA_06_ADDR   = 0x4F;
    static constexpr uint8_t EXT_SENS_DATA_07_ADDR   = 0x50;
    static constexpr uint8_t EXT_SENS_DATA_08_ADDR   = 0x51;
    static constexpr uint8_t EXT_SENS_DATA_09_ADDR   = 0x52;
    static constexpr uint8_t EXT_SENS_DATA_10_ADDR   = 0x53;
    static constexpr uint8_t EXT_SENS_DATA_11_ADDR   = 0x54;
    static constexpr uint8_t EXT_SENS_DATA_12_ADDR   = 0x55;
    static constexpr uint8_t EXT_SENS_DATA_13_ADDR   = 0x56;
    static constexpr uint8_t EXT_SENS_DATA_14_ADDR   = 0x57;
    static constexpr uint8_t EXT_SENS_DATA_15_ADDR   = 0x58;
    static constexpr uint8_t EXT_SENS_DATA_16_ADDR   = 0x59;
    static constexpr uint8_t EXT_SENS_DATA_17_ADDR   = 0x5A;
    static constexpr uint8_t EXT_SENS_DATA_18_ADDR   = 0x5B;
    static constexpr uint8_t EXT_SENS_DATA_19_ADDR   = 0x5C;
    static constexpr uint8_t EXT_SENS_DATA_20_ADDR   = 0x5D;
    static constexpr uint8_t EXT_SENS_DATA_21_ADDR   = 0x5E;
    static constexpr uint8_t EXT_SENS_DATA_22_ADDR   = 0x5F;
    static constexpr uint8_t EXT_SENS_DATA_23_ADDR   = 0x60;
    static constexpr uint8_t MOT_DETECT_STATUS_ADDR  = 0x61;
    static constexpr uint8_t I2C_SLV0_DO_ADDR        = 0x63;
    static constexpr uint8_t I2C_SLV1_DO_ADDR        = 0x64;
    static constexpr uint8_t I2C_SLV2_DO_ADDR        = 0x65;
    static constexpr uint8_t I2C_SLV3_DO_ADDR        = 0x66;
    static constexpr uint8_t I2C_MST_DELAY_CTRL_ADDR = 0x67;

    static constexpr uint8_t SIGNAL_PATH_RESET_ADDR = 0x68;
    static constexpr uint8_t GYRO_RESET_BIT         = BIT(2);
    static constexpr uint8_t ACC_RESET_BIT          = BIT(1);
    static constexpr uint8_t TEMP_RESET_BIT         = BIT(0);

    static constexpr uint8_t MOT_DETECT_CTRL_ADDR = 0x69;

    static constexpr uint8_t USER_CTRL_ADDR         = 0x6A;
    static constexpr uint8_t FIFO_ENABLED_BIT       = BIT(6);
    static constexpr uint8_t I2C_MASTER_ENABLED_BIT = BIT(5);
    static constexpr uint8_t I2C_ENABLED_BIT        = BIT(4);
    static constexpr uint8_t FIFO_RESET_BIT         = BIT(2);
    static constexpr uint8_t I2C_MASTER_RESET_BIT   = BIT(1);
    static constexpr uint8_t SIGNAL_COND_RESET_BIT  = BIT(0);

    static constexpr uint8_t PWR_MGMT_1_ADDR   = 0x6B;
    static constexpr uint8_t RESET_BIT         = BIT(7);
    static constexpr uint8_t SLEEP_BIT         = BIT(6);
    static constexpr uint8_t CYCLE_BIT         = BIT(5);
    static constexpr uint8_t TEMP_DISABLED_BIT = BIT(3);
    static constexpr uint8_t CLK_SEL_BITS      = 0b00000111;
    static constexpr uint8_t CLK_SEL_OFFSET    = 0;

    static constexpr uint8_t PWR_MGMT_2_ADDR   = 0x6C;
    static constexpr uint8_t LP_WAKE_CTRL_BITS = 0xC0;
    static constexpr uint8_t STBY_XA_BIT       = BIT(5);
    static constexpr uint8_t STBY_YA_BIT       = BIT(4);
    static constexpr uint8_t STBY_ZA_BIT       = BIT(3);
    static constexpr uint8_t STBY_XG_BIT       = BIT(2);
    static constexpr uint8_t STBY_YG_BIT       = BIT(1);
    static constexpr uint8_t STBY_ZG_BIT       = BIT(0);

    static constexpr uint8_t BANK_SEL_ADDR       = 0x6D;
    static constexpr uint8_t MEM_START_ADDR_ADDR = 0x6E;
    static constexpr uint8_t MEM_R_W_ADDR        = 0x6F;
    static constexpr uint8_t DMP_CFG_1_ADDR      = 0x70;
    static constexpr uint8_t DMP_CFG_2_ADDR      = 0x71;

    static constexpr uint8_t FIFO_COUNT_H_ADDR = 0x72;

    static constexpr uint8_t FIFO_COUNT_L_ADDR = 0x73;

    static constexpr uint8_t FIFO_DATA_ADDR = 0x74;

    static constexpr uint8_t WHO_AM_I_ADDR = 0x75;
    static constexpr uint8_t WHO_AM_I_BITS = 0x7E;

    static constexpr float SELF_TEST_ACC_THR   = MPU6050_SELF_TEST_ACC_THR;
    static constexpr float SELF_TEST_GYRO_THR  = MPU6050_SELF_TEST_GYRO_THR;
    static constexpr uint8_t SELF_TEST_SLEEP   = MPU6050_SELF_TEST_SLEEP;
    static constexpr uint8_t SELF_TEST_SAMPLES = MPU6050_SELF_TEST_SAMPLES;

    static constexpr uint64_t GYRO_CALIB_THR     = MPU6050_GYRO_CALIB_THR;
    static constexpr uint64_t GYRO_CALIB_SLEEP   = MPU6050_GYRO_CALIB_SLEEP;
    static constexpr uint16_t GYRO_CALIB_SAMPLES = MPU6050_GYRO_CALIB_SAMPLES;
    
    static constexpr uint8_t I2C_ADDR_AD0_LOW = 0x68;
    static constexpr uint8_t I2C_ADDR_AD0_HIGH = 0x69;
    // clang-format on

    /************************************************************************\
    |                                ENUMS                                   |
    \************************************************************************/

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
        _250 = 0,   //<! 4.36332 rad/s
        _500 = 1,   //<! 8.72665 rad/s
        _1000 = 2,  //<! 17.4533 rad/s
        _2000 = 3,  //<! 34.90659 rad/s
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

    /************************************************************************\
    |                              FUNCTIONS                                 |
    \************************************************************************/

    MPU6050(i2c_inst_t *i2c_inst, uint8_t address);

    inline int getAcc(Eigen::Vector<float, 3> &);
    inline int getRawAcc(Eigen::Vector<int16_t, 3> &);
    int setAccRange(AccRange);
    int getAccRange(AccRange &);
    int enableAccSelfTest();
    int disableAccSelfTest();
    int setAccOffset(const int16_t[3]);
    int getAccOffset(int16_t[3]);
    int setAccScale(const int8_t[3]);
    int getAccScale(int8_t[3]);

    inline int getRawGyro(Eigen::Vector<int16_t, 3> &);
    inline int getGyro(Eigen::Vector<float, 3> &);
    int setGyroRange(GyroRange);
    int getGyroRange(GyroRange &);
    int enableGyroSelfTest();
    int disableGyroSelfTest();
    void setGyroRad();
    void setGyroDeg();
    int calibrateGyro();
    int setGyroOffset(const int16_t[3]);
    int getGyroOffset(int16_t[3]);
    int setGyroScale(const int8_t[3]);
    int getGyroScale(int8_t[3]);

    inline int getAccGyro(Eigen::Vector<float, 3> &, Eigen::Vector<float, 3> &);
    inline int getRawAccGyro(Eigen::Vector<int16_t, 3> &,
                             Eigen::Vector<int16_t, 3> &);

    int getDLPFConfig(DlpfBW &);
    int setDLPFConfig(DlpfBW);

    int selfTest();
    int accSelfTest();
    int gyroSelfTest();

    int enableInterrupt();
    int getInterruptStatus(uint8_t &);

    int setClockSource(ClockSource);
    int getClockSource(ClockSource &);

    int dumpMemory(uint8_t, uint8_t);
    int dumpMemory(uint8_t);
    int dumpMemory();
    int reset();
    int reset_paths();
    int sleep();
    int wake();
    bool ok() { return !m_error; }

    /************************************************************************\
    |                               MEMBERS                                  |
    \************************************************************************/

private:
    arc::common::I2C m_i2c;
    bool m_I2C_master_mode{false};
    bool m_I2C_enabled{false};

    float m_acc_scale{1.0};

    float m_gyro_scale{1.0};
    bool m_gyro_in_rad{false};
    bool m_gyro_calibrated{false};

    bool m_error{false};
};

int MPU6050::getAcc(Eigen::Vector<float, 3> &accel) {
    uint8_t buf[6];
    int ret = m_i2c.readBytes(ACC_X_H_ADDR, buf, 6);
    if (ret != 6) {
        log_error << "Error in readBytes().\n";
        return EIO;
    }

    accel.x() = static_cast<int16_t>((buf[0] << 8) | buf[1]) * m_acc_scale;
    accel.y() = static_cast<int16_t>((buf[2] << 8) | buf[3]) * m_acc_scale;
    accel.z() = static_cast<int16_t>((buf[4] << 8) | buf[5]) * m_acc_scale;

    return 0;
}

int MPU6050::getRawAcc(Eigen::Vector<int16_t, 3> &accel) {
    uint8_t buf[6];
    int ret = m_i2c.readBytes(ACC_X_H_ADDR, buf, 6);
    if (ret != 6) {
        log_error << "Error in readBytes().\n";
        return EIO;
    }

    accel.x() = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    accel.y() = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    accel.z() = static_cast<int16_t>((buf[4] << 8) | buf[5]);

    return 0;
}

int MPU6050::getGyro(Eigen::Vector<float, 3> &gyro) {
    uint8_t buf[6];
    int ret = m_i2c.readBytes(GYRO_X_H_ADDR, buf, 6);
    if (ret != 6) {
        log_error << "Error in readBytes().\n";
        return EIO;
    }

    gyro.x() = (static_cast<int16_t>((buf[0] << 8) | buf[1])) * m_gyro_scale;
    gyro.y() = (static_cast<int16_t>((buf[2] << 8) | buf[3])) * m_gyro_scale;
    gyro.z() = (static_cast<int16_t>((buf[4] << 8) | buf[5])) * m_gyro_scale;

    return 0;
}

int MPU6050::getRawGyro(Eigen::Vector<int16_t, 3> &gyro) {
    uint8_t buf[6];
    int ret = m_i2c.readBytes(GYRO_X_H_ADDR, buf, 6);
    if (ret != 6) {
        log_error << "Error in readBytes().\n";
        return EIO;
    }

    gyro.x() = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    gyro.y() = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    gyro.z() = static_cast<int16_t>((buf[4] << 8) | buf[5]);

    return 0;
}

int MPU6050::getAccGyro(Eigen::Vector<float, 3> &accel,
                        Eigen::Vector<float, 3> &gyro) {
    uint8_t buf[14];
    int ret = m_i2c.readBytes(ACC_X_H_ADDR, buf, 14);
    if (ret != 14) {
        log_error << "Error in readBytes().\n";
        return EIO;
    }

    accel.x() = static_cast<int16_t>((buf[0] << 8) | buf[1]) * m_acc_scale;
    accel.y() = static_cast<int16_t>((buf[2] << 8) | buf[3]) * m_acc_scale;
    accel.z() = static_cast<int16_t>((buf[4] << 8) | buf[5]) * m_acc_scale;

    gyro.x() = (static_cast<int16_t>((buf[8] << 8) | buf[9])) * m_gyro_scale;
    gyro.y() = (static_cast<int16_t>((buf[10] << 8) | buf[11])) * m_gyro_scale;
    gyro.z() = (static_cast<int16_t>((buf[12] << 8) | buf[13])) * m_gyro_scale;

    return 0;
}

int MPU6050::getRawAccGyro(Eigen::Vector<int16_t, 3> &accel,
                           Eigen::Vector<int16_t, 3> &gyro) {
    uint8_t buf[14];
    int ret = m_i2c.readBytes(ACC_X_H_ADDR, buf, 14);
    if (ret != 14) {
        log_error << "Error in readBytes().\n";
        return EIO;
    }

    accel.x() = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    accel.y() = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    accel.z() = static_cast<int16_t>((buf[4] << 8) | buf[5]);

    gyro.x() = (static_cast<int16_t>((buf[8] << 8) | buf[9]));
    gyro.y() = (static_cast<int16_t>((buf[10] << 8) | buf[11]));
    gyro.z() = (static_cast<int16_t>((buf[12] << 8) | buf[13]));

    return 0;
}

}  // namespace arc::sensors
