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
    mpu6050.selfTest();
    // if (mpu6050.ok()) {
    //     mpu6050.calibrateGyro();
    // } else {
    //     return 0;
    // }

    mpu6050.setDLPFConfig(MPU6050::DlpfBW::_260Hz);
    mpu6050.setAccRange(MPU6050::AccRange::_8G);
    mpu6050.setGyroRange(MPU6050::GyroRange::_500);
    mpu6050.setClockSource(MPU6050::ClockSource::PLL_GYROX);

    int16_t acc_offset[3] = {-4603, -928, 1047};
    mpu6050.setAccOffset(acc_offset);
    int8_t acc_scale[3] = {5, -4, -3};
    mpu6050.setAccScale(acc_scale);

    int16_t gyro_offset[3] = {334, 21, -7};
    mpu6050.setGyroOffset(gyro_offset);

    sleep_ms(100);

    log_info << "\n";
    log_info << "START\n";
    log_info << "\n";

    sleep_ms(1000);

    size_t n = 1000;
    Vector<float, 3> acc[n];
    Vector<float, 3> gyro[n];
    uint64_t timestamp[n];

    for (size_t i = 0; i < n; i++) {
        mpu6050.getAccGyro(acc[i], gyro[i]);
        timestamp[i] = time_us_64();
        sleep_ms(10);
    }

    for (size_t i = 0; i < n; i++) {
        std::cout << timestamp[i] << "," << acc[i].x() << "," << acc[i].y()
                  << "," << acc[i].z() << "," << gyro[i].x() << ","
                  << gyro[i].y() << "," << gyro[i].z() << "\n";
    }

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
