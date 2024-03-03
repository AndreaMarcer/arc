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

#include <stdio.h>
#include <iostream>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "math.h"
#include "hardware/xosc.h"

#include "common/log.hpp"
#include "common/common.hpp"
#include "common/stopwatch.hpp"
#include "Eigen/Core"
// #include "sensors/MPU6050.hpp"

// uint32_t cnt = 0;
// bool new_data = false;
// Stopwatch stopwatch;

// void gpio_callback(uint gpio, uint32_t events)
// {
// 	cnt++;
// 	new_data = true;
// 	stopwatch.start();
// }

int main() {
    stdio_init_all();

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a
    // Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // gpio_set_irq_enabled_with_callback(9, GPIO_IRQ_EDGE_RISE, true,
    // 				   &gpio_callback);

    log_info("=======================\n");

    // MPU6050 mpu6050{ i2c_default, MPU6050::I2C_ADDR_AD0_LOW };
    // mpu6050.setDLPFConfig(MPU6050::DlpfBW::_184Hz);
    // mpu6050.setAccRange(MPU6050::AccRange::_2G);
    // mpu6050.enableInterrupt();
    // mpu6050.wake();

    // sleep_ms(100);

    // arc::math::Matrix<float, 3, 1> acc(0);
    // arc::math::Matrix<float, 3, 1> prev_acc(0);
    // float acc[3]{ 0 };
    // float prev_acc[3]{ 0 };
    // uint8_t tmp;
    // uint64_t cnt{ 0 };
    // Stopwatch stopwatch;
    // while (1) {
    // 	stopwatch.start();
    // 	mpu6050.getInterruptStatus(tmp);

    // 	gpio_set_dormant_irq_enabled(
    // 		9, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_HIGH_BITS,
    // 		true);

    // 	xosc_dormant();
    // 	stopwatch.stop().print().reset();

    // 	cnt++;
    // 	mpu6050.getAcc(acc);
    // 	for (uint8_t i = 0; i < 3; i++) {
    // 		acc[i] = 0.5f * acc[i] + 0.5f * prev_acc[i];
    // 		prev_acc[i] = acc[i];
    // 	}
    // 	// acc = 0.9f * acc + 0.1f * prev_acc;
    // 	// prev_acc = acc;
    // 	mpu6050.printAcc(acc);

    // 	sleep_ms(500);
    // }
}
