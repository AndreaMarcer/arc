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
static constexpr uint32_t NUM_SAMPLES = 150;
static constexpr uint32_t NUM_RUNS = 25;
static constexpr uint32_t MS_BETWEEN_SAMPLES = 20;
static constexpr uint32_t MS_PER_ACQUISITION = NUM_SAMPLES * MS_BETWEEN_SAMPLES;
static constexpr double ACC_STD_TRH = 1.4;
static constexpr double GYRO_STD_TRH = 0.40;

static constexpr int16_t ACC_OFFSET[3] = {0, 0, 0};
static constexpr int8_t ACC_SCALE[3] = {-8, -8, -8};

static constexpr int16_t GYRO_OFFSET[3] = {0, 0, 0};
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

    //
    // INIT OFFSET
    //
    int16_t orig_acc_offsets[3];
    mpu6050.getAccOffset(orig_acc_offsets);
    log_info << "orig_acc_offsets: " << orig_acc_offsets[0] << ", "
             << orig_acc_offsets[1] << ", " << orig_acc_offsets[2] << std::endl;

    int16_t orig_gyro_offsets[3];
    mpu6050.getGyroOffset(orig_gyro_offsets);
    log_info << "orig_gyro_offsets: " << orig_gyro_offsets[0] << ", "
             << orig_gyro_offsets[1] << ", " << orig_gyro_offsets[2]
             << std::endl;

    mpu6050.setAccOffset(ACC_OFFSET);
    mpu6050.setGyroOffset(GYRO_OFFSET);

    int16_t curr_acc_offsets[3];
    mpu6050.getAccOffset(curr_acc_offsets);
    log_info << "acc_offset: " << curr_acc_offsets[0] << ", "
             << curr_acc_offsets[1] << ", " << curr_acc_offsets[2] << std::endl;

    int16_t curr_gyro_offsets[3];
    mpu6050.getGyroOffset(curr_gyro_offsets);
    log_info << "gyro_offset: " << curr_gyro_offsets[0] << ", "
             << curr_gyro_offsets[1] << ", " << curr_gyro_offsets[2]
             << std::endl;

    //
    // INIT SCALE
    //
    int8_t orig_acc_scale[3];
    mpu6050.getAccScale(orig_acc_scale);
    log_info << "orig_acc_scale: " << signed(orig_acc_scale[0]) << ", "
             << signed(orig_acc_scale[1]) << ", " << signed(orig_acc_scale[2])
             << std::endl;

    int8_t orig_gyro_scale[3];
    mpu6050.getGyroScale(orig_gyro_scale);
    log_info << "orig_gyro_scale: " << (int)orig_gyro_scale[0] << ", "
             << (int)orig_gyro_scale[1] << ", " << (int)orig_gyro_scale[2]
             << std::endl;

    mpu6050.setAccScale(ACC_SCALE);
    // mpu6050.setGyroScale(GYRO_SCALE);

    int8_t curr_acc_scale[3];
    mpu6050.getAccScale(curr_acc_scale);
    log_info << "curr_acc_scale: " << (int)curr_acc_scale[0] << ", "
             << (int)curr_acc_scale[1] << ", " << (int)curr_acc_scale[2]
             << std::endl;

    int8_t curr_gyro_scale[3];
    mpu6050.getGyroScale(curr_gyro_scale);
    log_info << "curr_gyro_scale: " << (int)curr_gyro_scale[0] << ", "
             << (int)curr_gyro_scale[1] << ", " << (int)curr_gyro_scale[2]
             << std::endl;

    //
    // RUNS
    //
    sleep_ms(100);

    Vector<int16_t, 3> acc[NUM_RUNS][NUM_SAMPLES];
    Vector<int16_t, 3> gyro[NUM_RUNS][NUM_SAMPLES];
    uint64_t timestamp[NUM_RUNS][NUM_SAMPLES];

    bool finished = false;
    uint32_t num_run = 0;
    log_info << "\n";
    while (!finished) {
        log_info << "RUN [" << num_run + 1 << " / " << NUM_RUNS << "]\n";
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

        if ((acc_std.array() > ACC_STD_TRH).any()) {
            log_info << " - acc_std: " << acc_std.transpose() << "\n";
            log_info << " - ACC STD toot high\n";
            continue;
        }

        if ((gyro_std.array() > GYRO_STD_TRH).any()) {
            log_info << " - gyro_std: " << gyro_std.transpose() << "\n";
            log_info << " - GYRO STD toot high\n";
            continue;
        }

        log_info << " - acc_mean: " << acc_mean.transpose() << "\n";
        log_info << " - gyro_mean: " << gyro_mean.transpose() << "\n";
        log_info << " - acc_std: " << acc_std.transpose() << "\n";
        log_info << " - gyro_std: " << gyro_std.transpose() << "\n";

        num_run++;
        if (num_run == NUM_RUNS) finished = true;
    }
    log_info << " \n";
    log_info << " CALIBRATION DONE \n\n\n\n\n\n";

    //
    // PRINT
    //

    std::cout << 0 << "," << curr_acc_offsets[0] << "," << curr_acc_offsets[1]
              << "," << curr_acc_offsets[2] << "," << curr_gyro_offsets[0]
              << "," << curr_gyro_offsets[1] << "," << curr_gyro_offsets[2]
              << "\n";

    std::cout << 0 << "," << (int32_t)curr_acc_scale[0] << ","
              << (int32_t)curr_acc_scale[1] << "," << (int32_t)curr_acc_scale[2]
              << "," << (int32_t)curr_gyro_scale[0] << ","
              << (int32_t)curr_gyro_scale[1] << ","
              << (int32_t)curr_gyro_scale[2] << "\n";

    for (uint32_t run = 0; run < NUM_RUNS; run++) {
        for (uint32_t sample = 0; sample < NUM_SAMPLES; sample++) {
            std::cout << timestamp[run][sample] << "," << acc[run][sample].x()
                      << "," << acc[run][sample].y() << ","
                      << acc[run][sample].z() << "," << gyro[run][sample].x()
                      << "," << gyro[run][sample].y() << ","
                      << gyro[run][sample].z() << "\n";
        }
    }

    return 0;
}
