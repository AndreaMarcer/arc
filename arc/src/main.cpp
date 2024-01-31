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

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "math.h"

#include "common/log.hpp"
#include "common/common.hpp"
#include "common/geometry.hpp"
#include "math/matrix.hpp"
#include "sensors/MPU6050.hpp"

using namespace arc::sensors;
using namespace arc::common;

uint32_t cnt = 0;
bool new_data = false;
uint64_t interrupt_time = 0;

void gpio_callback(uint gpio, uint32_t events)
{
	cnt++;
	new_data = true;
	interrupt_time = time_us_64();
}

int main()
{
	stdio_init_all();

	// This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
	i2c_init(i2c_default, 400 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

	gpio_set_irq_enabled_with_callback(9, GPIO_IRQ_EDGE_RISE, true,
					   &gpio_callback);

	log_info("=======================\n");

	arc::math::Matrix<int, 2, 3> m1 = arc::math::Matrix<int, 2, 3>();
	m1.print();
	m1.clear();
	m1.print();

	arc::math::Matrix<uint, 3, 2> m2;
	m2.print();

	float arr[1][2] = { { 1, 2 } };
	arc::math::Matrix m3(arr);
	m3.print();

	arc::math::Matrix<int, 2, 3> m4;
	m4.clear();
	m4.print();

	arc::math::Matrix<int, 2, 3> m12{ { 1, 2, 3 }, { 2, 5, 6 } };
	m12.print();

	m4[1][1] = 5;

	arc::math::Matrix m5(m4);
	m5.print();

	arc::math::Matrix m6 = m4 * m5;
	m6.print();

	arc::math::Matrix m7 = m6 * 2;
	m7.print();

	int num = 2;
	arc::math::Matrix m8 = m6 * num;
	m8.print();

	m7 = m8 * 2;
	m7.print();

	arc::math::Matrix<int, 2, 3> m9(1);
	m9.print();

	arc::math::Matrix<int, 3, 2> m10(3);
	m10.print();

	arc::math::Matrix m11 = m9 * m10;
	m11.print();

	// arc::common::Vector<int, 3> v1 = { 2 };
	// arc::common::Vector<int, 3> v2 = { 1 };
	// arc::common::Vector<int, 3> v3;

	// print(v1);
	// print(v2);
	// print(v3);

	// if (v1 == v2) {
	// 	printf("EQUAL\n");
	// }
	// v3 = v1 + v2;
	// print(v3);

	// MPU6050 mpu6050{ i2c_default, MPU6050::I2C_ADDR_AD0_LOW };
	// mpu6050.setDLPFConfig(MPU6050::DlpfBW::_184Hz);
	// mpu6050.setAccRange(MPU6050::AccRange::_2G);
	// mpu6050.enableInterrupt();
	// mpu6050.wake();

	// sleep_ms(100);

	// arc::common::acc_t acc;
	// arc::common::acc_t gyro;
	// uint8_t tmp;
	// while (0) {
	// 	if (new_data) {
	// 		mpu6050.getAcc(acc.vec);
	// 		mpu6050.getGyro(gyro.vec);
	// 		printf("%u, %f,%f,%f %f,%f,%f %lu\n", cnt, acc.x, acc.y,
	// 		       acc.z, gyro.x, gyro.y, gyro.z,
	// 		       time_us_64() - interrupt_time);
	// 		new_data = false;
	// 		sleep_ms(1000);
	// 		mpu6050.getInterruptStatus(tmp);
	// 	}
	// }
}
