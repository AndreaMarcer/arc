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
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "math.h"
#include "hardware/gpio.h"
#include "sensors/MPU6050.hpp"

uint32_t cnt = 0;

uint8_t int_status_addr = MPU6050_INT_STATUS_ADDR;
uint8_t buf;
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

	printf("\n\n\n\nHello, MPU6050! Reading raw data from registers...\n");

	// This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
	i2c_init(i2c_default, 400 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
	// Make the I2C pins available to picotool
	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN,
				   PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

	gpio_set_irq_enabled_with_callback(9, GPIO_IRQ_EDGE_RISE, true,
					   &gpio_callback);

	arc::sensors::MPU6050 mpu6050{ 0x68 };
	mpu6050.setDLPFConfig(arc::sensors::MPU6050_DLPF_184HZ);
	mpu6050.enableInterrupt();
	mpu6050.wake();

	sleep_ms(100);

	float acc[3];
	uint8_t tmp;
	while (1) {
		if (new_data) {
			mpu6050.getAcc(acc);
			printf("%u, %f,%f,%f %lu\n", cnt, acc[0], acc[1],
			       acc[2], time_us_64() - interrupt_time);

			new_data = false;
			sleep_ms(100);
			mpu6050.getInterruptStatus(tmp);
		}
	}
}
