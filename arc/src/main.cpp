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

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/xosc.h"
#include "math.h"
#include "pico/stdlib.h"

#include "common/log.hpp"
#include "common/common.hpp"
#include "common/stopwatch.hpp"
#include "control/kalman.hpp"
#include "sensors/MPU6050.hpp"

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
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    log_info << "=======================\n";

    MPU6050 mpu6050{i2c_default, MPU6050::I2C_ADDR_AD0_LOW};
    mpu6050.wake();

    mpu6050.setDLPFConfig(MPU6050::DlpfBW::_184Hz);
    mpu6050.setAccRange(MPU6050::AccRange::_4G);
    mpu6050.setGyroRange(MPU6050::GyroRange::_500);
    mpu6050.setClockSource(MPU6050::ClockSource::INTR_8MHZ);

    // mpu6050.enableInterrupt();

    sleep_ms(100);

    int n = 10;
    Vector<float, 3> acc[n];
    Vector<float, 3> gyro[n];
    uint64_t timestamp[n];
    Stopwatch s1;
    // uint8_t tmp;
    for (size_t i = 0; i < n; i++) {
        // mpu6050.getInterruptStatus(tmp);

        // gpio_set_dormant_irq_enabled(
        //     9, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_HIGH_BITS, true);

        // xosc_dormant();
        s1.start();
        mpu6050.getAcc(acc[i]);
        mpu6050.getGyro(gyro[i]);
        s1.stop().print().reset();

        s1.start();
        mpu6050.getAccGyro(acc[i], gyro[i]);
        s1.stop().print().reset();
        timestamp[i] = time_us_64();
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
