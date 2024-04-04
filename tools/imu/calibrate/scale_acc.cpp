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
#define OFFSET_START -4000
#define OFFSET_STEP 80
#define OFFSET_RANGE 100

#define SLEEP_MS_TEST 5

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
    mpu6050.setGyroRange(MPU6050::GyroRange::_500);
    mpu6050.setClockSource(MPU6050::ClockSource::PLL_GYROX);

    int16_t orig_acc_offsets[3];
    mpu6050.getAccOffset(orig_acc_offsets);
    log_info << "orig_acc_offsets: " << orig_acc_offsets[0] << ", "
             << orig_acc_offsets[1] << ", " << orig_acc_offsets[2] << std::endl;

    int8_t orig_acc_scale[3];
    mpu6050.getAccScale(orig_acc_scale);
    log_info << "orig_acc_scale: " << (int)orig_acc_scale[0] << ", "
             << (int)orig_acc_scale[1] << ", " << (int)orig_acc_scale[2]
             << std::endl;

    log_info << "\n";
    log_info << "START\n";
    log_info << "\n";

    sleep_ms(1000);

    int n = SCALE_RANGE * OFFSET_RANGE;
    Vector<int16_t, 3> acc_raw[n];
    int32_t scales[n];
    int32_t offsets[n];

    int16_t offset[3] = {OFFSET_START, OFFSET_START, OFFSET_START};
    mpu6050.setAccOffset(offset);

    int8_t scale[3] = {SCALE_START, SCALE_START, SCALE_START};
    mpu6050.setAccScale(scale);

    for (int j = 0; j < SCALE_RANGE; j++) {
        for (int i = 0; i < OFFSET_RANGE; i++) {
            log_info << "SCALE: " << signed(scale[0])
                     << "\tOFFSET: " << signed(offset[0]) << "\n";

            mpu6050.getRawAcc(acc_raw[i + j * OFFSET_RANGE]);
            scales[i + j * OFFSET_RANGE] = scale[0];
            offsets[i + j * OFFSET_RANGE] = offset[0];

            offset[0] += OFFSET_STEP;
            offset[1] += OFFSET_STEP;
            offset[2] += OFFSET_STEP;
            mpu6050.setAccOffset(offset);

            sleep_ms(SLEEP_MS_TEST);
        }

        offset[0] = OFFSET_START;
        offset[1] = OFFSET_START;
        offset[2] = OFFSET_START;
        mpu6050.setAccOffset(offset);

        scale[0] += SCALE_STEP;
        scale[1] += SCALE_STEP;
        scale[2] += SCALE_STEP;
        mpu6050.setAccScale(scale);
        sleep_ms(SLEEP_MS_TEST);
    }

    for (int i = 0; i < n; i++) {
        std::cout << signed(scales[i]) << "," << signed(offsets[i]) << ","
                  << signed(acc_raw[i].x()) << "," << signed(acc_raw[i].y())
                  << "," << signed(acc_raw[i].z()) << "\n";
    }

    return 0;
}
