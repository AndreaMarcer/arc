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
static constexpr uint32_t NUM_SAMPLES = 100;
static constexpr uint32_t NUM_RUNS = 1;
static constexpr uint32_t MS_BETWEEN_SAMPLES = 50;
static constexpr uint32_t MS_PER_ACQUISITION = NUM_SAMPLES * MS_BETWEEN_SAMPLES;
static constexpr double ACC_STD_TRH = 10.0;
static constexpr double GYRO_STD_TRH = 2.0;

/*****************************************************************************\
|                                     MAIN                                    |
\*****************************************************************************/
int main() {
    using namespace arc::sensors;
    using namespace arc::common;
    using namespace Eigen;

    stdio_init_all();

    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    log_info << "================================================\n";

    MPU6050 mpu6050{i2c_default, MPU6050::I2C_ADDR_AD0_LOW};

    mpu6050.wake();
    mpu6050.setDLPFConfig(MPU6050::DlpfBW::_260Hz);
    mpu6050.setAccRange(MPU6050::AccRange::_2G);
    mpu6050.setGyroRange(MPU6050::GyroRange::_250);
    mpu6050.setClockSource(MPU6050::ClockSource::PLL_GYROX);
    sleep_ms(100);

    int16_t curr_acc_offsets[3];
    mpu6050.getAccOffset(curr_acc_offsets);
    log_info << "curr_acc_offset: " << curr_acc_offsets[0] << ", "
             << curr_acc_offsets[1] << ", " << curr_acc_offsets[2] << std::endl;

    int16_t curr_gyro_offsets[3];
    mpu6050.getGyroOffset(curr_gyro_offsets);
    log_info << "curr_gyro_offset: " << curr_gyro_offsets[0] << ", "
             << curr_gyro_offsets[1] << ", " << curr_gyro_offsets[2]
             << std::endl;

    //
    // RUNS
    //
    Vector<int16_t, 3> acc[NUM_RUNS][NUM_SAMPLES];
    Vector<int16_t, 3> gyro[NUM_RUNS][NUM_SAMPLES];
    uint64_t timestamp[NUM_RUNS][NUM_SAMPLES];

    bool finished = false;
    uint32_t num_run = 0;
    log_info << "\n";
    while (!finished) {
        log_info << "RUN [" << num_run << "]\n";
        log_info << " - Start";
        for (uint32_t i = 0; i < NUM_SAMPLES; i++) {
            mpu6050.getRawAccGyro(acc[num_run][i], gyro[num_run][i]);
            timestamp[num_run][i] = time_us_64();

            sleep_ms(MS_BETWEEN_SAMPLES);
        }
        log_info_s << "-> DONE\n";

        //
        // MEAN
        //
        Vector<int64_t, 3> acc_mean = Vector<int64_t, 3>::Zero();
        Vector<int64_t, 3> gyro_mean = Vector<int64_t, 3>::Zero();
        for (uint32_t i = 0; i < NUM_SAMPLES; i++) {
            acc_mean += acc[num_run][i].cast<int64_t>();
            gyro_mean += gyro[num_run][i].cast<int64_t>();
        }
        acc_mean /= NUM_SAMPLES;
        gyro_mean /= NUM_SAMPLES;
        log_info << " - acc_mean: " << acc_mean.transpose() << "\n";
        log_info << " - gyro_mean: " << gyro_mean.transpose() << "\n";

        //
        // STD
        //
        Vector<double, 3> acc_std = Vector<double, 3>::Zero();
        Vector<double, 3> gyro_std = Vector<double, 3>::Zero();
        Vector<int64_t, 3> acc_temp;
        Vector<int64_t, 3> gyro_temp;
        for (uint32_t i = 0; i < NUM_SAMPLES; i++) {
            acc_temp =
                (acc[num_run][i].cast<int64_t>() - acc_mean).array().pow(2);
            acc_std += acc_temp.cast<double>();
            gyro_temp =
                (gyro[num_run][i].cast<int64_t>() - gyro_mean).array().pow(2);
            gyro_std += gyro_temp.cast<double>();
        }
        acc_std = acc_std.array().sqrt() / (NUM_SAMPLES - 1);
        gyro_std = gyro_std.array().sqrt() / (NUM_SAMPLES - 1);

        log_info << " - acc_std: " << acc_std.transpose() << "\n";
        log_info << " - gyro_std: " << gyro_std.transpose() << "\n";

        if ((acc_std.array() > ACC_STD_TRH).any()) {
            log_info << " - ACC STD toot high\n";
            continue;
        }

        if ((gyro_std.array() > GYRO_STD_TRH).any()) {
            log_info << " - GYRO STD toot high\n";
            continue;
        }

        num_run++;
        if (num_run == NUM_RUNS) finished = true;
    }
    log_info << " \n";
    log_info << " CALIBRATION DONE \n";

    // for (size_t i = 0; i < n; i++) {
    //     std::cout << timestamp_1[i] << "," << acc_1[i].x() << ","
    //               << acc_1[i].y() << "," << acc_1[i].z() << "," <<
    //               gyro_1[i].x()
    //               << "," << gyro_1[i].y() << "," << gyro_1[i].z() << "\n";
    // }

    // int16_t offsets[3];
    // mpu6050.getAccOffset(offsets);

    // int16_t acc_offset[3] = {-4558, -949, 1040};
    // mpu6050.setAccOffset(acc_offset);
    // mpu6050.getAccOffset(offsets);
    // log_info << offsets[0] << ", " << offsets[1] << ", " << offsets[2] << ",
    // "
    //          << std::endl;

    // mpu6050.getGyroOffset(offsets);
    // log_info << offsets[0] << ", " << offsets[1] << ", " << offsets[2] << ",
    // "
    //          << std::endl;

    // int16_t acc_offset[3] = {  -42,   40,   48};
    // int16_t acc_offset[3] = {-4558, -949, 1040};
    //     mpu6050.dumpMemory(MPU6050::XA_OFFS_H_ADDR,
    //     MPU6050::ZA_OFFS_L_TC_ADDR); int16_t acc_offset[3] = {-4600,
    //     -909, 1088}; mpu6050.setAccOffset(acc_offset);
    //     mpu6050.dumpMemory(MPU6050::XA_OFFS_H_ADDR,
    //     MPU6050::ZA_OFFS_L_TC_ADDR);

    return 0;
}
