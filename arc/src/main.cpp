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
    mpu6050.setDLPFConfig(MPU6050::DlpfBW::_184Hz);
    mpu6050.setAccRange(MPU6050::AccRange::_2G);
    // mpu6050.enableInterrupt();
    mpu6050.wake();

    sleep_ms(100);

    Vector<float, 3> acc;
    Vector<float, 3> gyro;

    // uint8_t tmp;
    Stopwatch stopwatch;
    while (0) {
        // mpu6050.getInterruptStatus(tmp);

        // gpio_set_dormant_irq_enabled(
        //     9, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_HIGH_BITS, true);

        // xosc_dormant();

        stopwatch.start();
        mpu6050.getAcc(acc);
        mpu6050.getGyro(gyro);
        stopwatch.stop().print().reset();
        log_info << acc.transpose() << "\t|.| = " << acc.squaredNorm() << "\n";
        log_info << gyro.transpose() << "\n";

        sleep_ms(5000);
    }

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
