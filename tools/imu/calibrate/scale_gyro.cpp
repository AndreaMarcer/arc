/**
 * @file main.cpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-01-14
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

#include "common/log.hpp"
#include "common/common.hpp"
#include "common/stopwatch.hpp"
#include "control/kalman.hpp"
#include "sensors/MPU6050.hpp"

/*****************************************************************************\
|                                    MACRO                                    |
\*****************************************************************************/

#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13

#define SCALE_START -8
#define SCALE_STEP 1
#define SCALE_RANGE 16
#define OFFSET_START -5000
#define OFFSET_STEP 100
#define OFFSET_RANGE 100

#define SLEEP_MS_TEST 10

/*****************************************************************************\
|                                     MAIN                                    |
\*****************************************************************************/
int main() {
    using namespace arc::sensors;
    using namespace arc::common;
    using namespace arc::control;
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
    mpu6050.setAccRange(MPU6050::AccRange::_16G);
    mpu6050.setGyroRange(MPU6050::GyroRange::_2000);
    mpu6050.setClockSource(MPU6050::ClockSource::PLL_GYROX);

    int16_t orig_gyro_offsets[3];
    mpu6050.getGyroOffset(orig_gyro_offsets);
    log_info << "orig_gyro_offsets: " << orig_gyro_offsets[0] << ", "
             << orig_gyro_offsets[1] << ", " << orig_gyro_offsets[2]
             << std::endl;

    int8_t orig_gyro_scale[3];
    mpu6050.getGyroScale(orig_gyro_scale);
    log_info << "orig_gyro_scale: " << (int)orig_gyro_scale[0] << ", "
             << (int)orig_gyro_scale[1] << ", " << (int)orig_gyro_scale[2]
             << std::endl;

    log_info << "\n";
    log_info << "START\n";
    log_info << "\n";

    sleep_ms(1000);

    int n = SCALE_RANGE * OFFSET_STEP;
    Vector<int16_t, 3> gyro_raw[n];
    int32_t scales[n];
    int32_t offsets[n];

    int16_t offset[3] = {OFFSET_START, OFFSET_START, OFFSET_START};
    mpu6050.setGyroOffset(offset);

    int8_t scale[3] = {SCALE_START, SCALE_START, SCALE_START};
    mpu6050.setGyroScale(scale);

    for (int j = 0; j < SCALE_RANGE; j++) {
        for (int i = 0; i < OFFSET_STEP; i++) {
            log_info << "SCALE: " << signed(scale[0])
                     << " \t\tOFFSET: " << offset[0] << "\n";

            mpu6050.getRawGyro(gyro_raw[i + j * OFFSET_STEP]);
            scales[i + j * OFFSET_STEP] = scale[0];
            offsets[i + j * OFFSET_STEP] = offset[0];

            offset[0] += OFFSET_STEP;
            offset[1] += OFFSET_STEP;
            offset[2] += OFFSET_STEP;
            mpu6050.setGyroOffset(offset);

            sleep_ms(SLEEP_MS_TEST);
        }

        offset[0] = OFFSET_START;
        offset[1] = OFFSET_START;
        offset[2] = OFFSET_START;
        mpu6050.setGyroOffset(offset);

        scale[0] += SCALE_STEP;
        scale[1] += SCALE_STEP;
        scale[2] += SCALE_STEP;
        mpu6050.setGyroScale(scale);
        sleep_ms(SLEEP_MS_TEST);
    }

    for (int i = 0; i < n; i++) {
        std::cout << scales[i] << "," << offsets[i] << ","
                  << (int32_t)gyro_raw[i].x() << "," << (int32_t)gyro_raw[i].y()
                  << "," << (int32_t)gyro_raw[i].z() << "\n";
    }

    return 0;
}
