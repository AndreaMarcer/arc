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
#include "hardware/adc.h"
#include "math.h"

#include "hardware/gpio.h"

#include "sensors/MPU6050.hpp"

uint counter = 0;

arc::sensors::MPU6050 *p_mpu6050{ nullptr };
float g_acc[3];

void gpio_callback(uint gpio, uint32_t events)
{
	counter++;
}

int main()
{
	stdio_init_all();
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || \
	!defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/mpu6050_i2c example requires a board with I2C pins
	puts("Default I2C pins were not defined");
	return 0;
#else
	printf("\n\n\n\nHello, MPU6050! Reading raw data from registers...\n");

	const uint LED_PIN = PICO_DEFAULT_LED_PIN;

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	// This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
	i2c_init(i2c_default, 400 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
	// Make the I2C pins available to picotool
	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN,
				   PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

	arc::sensors::MPU6050 mpu6050{ 0x68 };
	p_mpu6050 = &mpu6050;

	sleep_ms(100);

	mpu6050.sleep();

#define PIN 9
	gpio_init(PIN);
	gpio_set_dir(PIN, GPIO_IN);
	gpio_set_irq_enabled_with_callback(PIN, GPIO_IRQ_EDGE_RISE, true,
					   &gpio_callback);
	sleep_ms(100);

	mpu6050.enableInterrupt();
	mpu6050.wake();

	while (1) {
		printf("Level: %d, counter: %d\n", gpio_get(PIN), counter);
		sleep_ms(1000);
	}
#endif
}
