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

	arc::sensors::MPU6050 mpu6050{ 0x68 };

	sleep_ms(100);

	float acc[3];
	uint32_t counter = 0;
	while (1) {
		mpu6050.getAcc(acc);
		sleep_ms(500);
		printf("%u,%f,%f,%f\n", counter++, acc[0], acc[1], acc[2]);
	}
}
