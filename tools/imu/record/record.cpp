/**
 * @file calibrate.cpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-21
 *
 * @copyright Copyright (c) 2024
 *
 */

/*****************************************************************************\
|                                   INCLUDES                                  |
\*****************************************************************************/
#include <stdio.h>
#include <iostream>

#include "math.h"
#include "pico/stdlib.h"

#include "Eigen/Eigen"

#include "common/log.hpp"
#include "common/common.hpp"
#include "common/stopwatch.hpp"
#include "sensors/MPU6050.hpp"

/*****************************************************************************\
|                                    MACRO                                    |
\*****************************************************************************/
static constexpr uint32_t NUM_SAMPLES = 10 * 100;
static constexpr uint32_t MS_BETWEEN_SAMPLES = 10;

static constexpr int16_t ACC_OFFSET[3] = {-4603, -928, 1047};
static constexpr int8_t ACC_SCALE[3] = {5, -4, -3};

static constexpr int16_t GYRO_OFFSET[3] = {334, 21, -7};
// static constexpr int8_t GYRO_SCALE[3] = {-8, -8, -8};

static constexpr uint I2C_SDA_PIN = 12;
static constexpr uint I2C_SCL_PIN = 13;

/*****************************************************************************\
|                                     MAIN                                    |
\*****************************************************************************/
int main() {
    using namespace arc::sensors;
    using namespace arc::common;
    using namespace Eigen;

    stdio_init_all();

    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    log_info << "================================================\n";

    MPU6050 mpu6050{i2c_default, MPU6050::I2C_ADDR_AD0_LOW};

    mpu6050.wake();

    mpu6050.setDLPFConfig(MPU6050::DlpfBW::_260Hz);
    mpu6050.setAccRange(MPU6050::AccRange::_8G);
    mpu6050.setGyroRange(MPU6050::GyroRange::_1000);
    mpu6050.setClockSource(MPU6050::ClockSource::PLL_GYROX);

    mpu6050.setAccOffset(ACC_OFFSET);
    mpu6050.setAccScale(ACC_SCALE);
    mpu6050.setGyroOffset(GYRO_OFFSET);
    // mpu6050.setGyroScale(GYRO_SCALE);

    log_info << "Recording for " << NUM_SAMPLES * MS_BETWEEN_SAMPLES / 1000.0f
             << " seconds\n";

    sleep_ms(100);

    Vector<float, 3> acc[NUM_SAMPLES];
    Vector<float, 3> gyro[NUM_SAMPLES];
    uint64_t timestamp[NUM_SAMPLES];
    for (uint32_t i = 0; i < NUM_SAMPLES; i++) {
        mpu6050.getAccGyro(acc[i], gyro[i]);
        timestamp[i] = time_us_64();
        sleep_ms(MS_BETWEEN_SAMPLES);
    }

    //
    // PRINT
    //

    int16_t acc_offset[3];
    mpu6050.getAccOffset(acc_offset);
    int16_t gyro_offset[3];
    mpu6050.getGyroOffset(gyro_offset);
    std::cout << 0 << "," << acc_offset[0] << "," << acc_offset[1] << ","
              << acc_offset[2] << "," << gyro_offset[0] << "," << gyro_offset[1]
              << "," << gyro_offset[2] << "\n";

    int8_t acc_scale[3];
    mpu6050.getAccScale(acc_scale);
    int8_t gyro_scale[3];
    mpu6050.getGyroScale(gyro_scale);
    std::cout << 0 << "," << signed(acc_scale[0]) << "," << signed(acc_scale[1])
              << "," << signed(acc_scale[2]) << "," << signed(gyro_scale[0])
              << "," << signed(gyro_scale[1]) << "," << signed(gyro_scale[2])
              << "\n";

    for (uint32_t i = 0; i < NUM_SAMPLES; i++) {
        std::cout << timestamp[i] << "," << acc[i].x() << "," << acc[i].y()
                  << "," << acc[i].z() << "," << gyro[i].x() << ","
                  << gyro[i].y() << "," << gyro[i].z() << "\n";
    }

    return 0;
}
