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

#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21

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

    MPU6050 mpu6050_1{i2c_default, MPU6050::I2C_ADDR_AD0_LOW};
    // MPU6050 mpu6050_2{i2c_default, MPU6050::I2C_ADDR_AD0_HIGH};

    mpu6050_1.wake();
    mpu6050_1.selfTest();
    if (mpu6050_1.ok()) {
        // mpu6050_1.calibrateGyro();
    } else {
        return 0;
    }

    mpu6050_1.setDLPFConfig(MPU6050::DlpfBW::_260Hz);
    mpu6050_1.setAccRange(MPU6050::AccRange::_4G);
    mpu6050_1.setGyroRange(MPU6050::GyroRange::_500);
    mpu6050_1.setClockSource(MPU6050::ClockSource::PLL_GYROX);

    int16_t acc_offset[3] = {-4709, -903, 866};
    mpu6050_1.setAccOffset(acc_offset);
    int16_t gyro_offset[3] = {334, 21, -7};
    mpu6050_1.setGyroOffset(gyro_offset);

    // mpu6050_2.wake();
    // mpu6050_2.setDLPFConfig(MPU6050::DlpfBW::_260Hz);
    // mpu6050_2.setAccRange(MPU6050::AccRange::_4G);
    // mpu6050_2.setGyroRange(MPU6050::GyroRange::_500);
    // mpu6050_2.setClockSource(MPU6050::ClockSource::PLL_GYROX);

    sleep_ms(100);

    log_info << "\n";
    log_info << "START\n";
    log_info << "\n";

    sleep_ms(1000);

    int n = 1000;
    Vector<float, 3> acc_1[n];
    Vector<float, 3> gyro_1[n];
    uint64_t timestamp_1[n];
    Vector<float, 3> acc_2[n];
    Vector<float, 3> gyro_2[n];
    uint64_t timestamp_2[n];

    for (size_t i = 0; i < n; i++) {
        mpu6050_1.getAccGyro(acc_1[i], gyro_1[i]);
        timestamp_1[i] = time_us_64();
        // mpu6050_2.getAccGyro(acc_2[i], gyro_2[i]);
        // timestamp_2[i] = time_us_64();
        sleep_ms(10);
    }
    for (size_t i = 0; i < n; i++) {
        std::cout << timestamp_1[i] << "," << acc_1[i].x() << ","
                  << acc_1[i].y() << "," << acc_1[i].z() << "," << gyro_1[i].x()
                  << "," << gyro_1[i].y() << "," << gyro_1[i].z() << "\n";
    }
    // std::cout << "SECOND\n";
    // for (size_t i = 0; i < n; i++) {
    //     std::cout << timestamp_2[i] << "," << acc_2[i].x() << ","
    //               << acc_2[i].y() << "," << acc_2[i].z() << "," <<
    //               gyro_2[i].x()
    //               << "," << gyro_2[i].y() << "," << gyro_2[i].z() << "\n";
    // }
    return 0;
    // clang-format off
    Matrix<float, 6, 6> A;
    A << 1.0, 1.0, 0.5, 0.0, 0.0, 0.0,
         0.0, 1.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 1.0, 0.5,
         0.0, 0.0, 0.0, 0.0, 1.0, 1.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    Matrix<float, 6, 6> P;
    P << 500.0,   0.0,   0.0,   0.0,   0.0,   0.0,
           0.0, 500.0,   0.0,   0.0,   0.0,   0.0,
           0.0,   0.0, 500.0,   0.0,   0.0,   0.0,
           0.0,   0.0,   0.0, 500.0,   0.0,   0.0,
           0.0,   0.0,   0.0,   0.0, 500.0,   0.0,
           0.0,   0.0,   0.0,   0.0,   0.0, 500.0;

    Matrix<float, 6, 6> Q;
    Q << 1/4, 1/2, 1/2, 0.0, 0.0, 0.0,
         1/2, 1.0, 1.0, 0.0, 0.0, 0.0,
         1/2, 1.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1/4, 1/2, 1/2,
         0.0, 0.0, 0.0, 1/2, 1.0, 1.0,
         0.0, 0.0, 0.0, 1/2, 1.0, 1.0;
    Q*=0.2*0.2;

    Matrix<float, 2, 2> R;
    R << 9.0, 0.0,
         0.0, 9.0;

    Matrix<float, 2, 6> H;
    H << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    
    Vector<float, 2> z;

    KF<6, 2, 1> kf(
        Vector<float, 6>::Zero(), 
        A, 
        Matrix<float, 6, 1>::Zero(), 
        P,
        Q, 
        R, 
        H
    );
    // clang-format on 

    Stopwatch sw;

    sw.start();
    kf.predict();
    sw.stop().print().reset();
    kf.printState();
    kf.printEstimateCovariance();
    
    z << 301.5, -401.46;
    sw.start();
    kf.update(z);
    sw.stop().print();
    kf.printState();
    kf.printEstimateCovariance();
    kf.printKalmanGain();

    sw.start();
    kf.predict();
    sw.stop().print().reset();
    kf.printState();
    kf.printEstimateCovariance();
}
