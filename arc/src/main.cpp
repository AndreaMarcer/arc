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

#include "Eigen/Core"

#include "common/log.hpp"
#include "common/common.hpp"
#include "common/stopwatch.hpp"
#include "sensors/MPU6050.hpp"

/*****************************************************************************\
|                                     MAIN                                    |
\*****************************************************************************/
int main() {
    using namespace arc::sensors;
    using namespace arc::common;

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

    Eigen::Vector<float, 3> acc;
    Eigen::Vector<float, 3> gyro;

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
}
