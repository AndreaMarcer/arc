/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "math.h"

#include "sensors/MPU6050.hpp"

/* Example code to talk to a MPU6050 MEMS accelerometer and gyroscope

   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (pin 6)) -> SDA on MPU6050 board
   GPIO PICO_DEFAULT_I2C_SCL_PIN (On Pico this is GP5 (pin 7)) -> SCL on MPU6050 board
   3.3v (pin 36) -> VCC on MPU6050 board
   GND (pin 38)  -> GND on MPU6050 board
*/

// By default these devices  are on bus address 0x68
static int addr = 0x68;

#ifdef i2c_default
static void mpu6050_reset()
{
	// Two byte reset. First byte register, second byte data
	// There are a load more options to set up the device in different ways that could be added here
	uint8_t buf[] = { 0x6B, 0x80 };
	i2c_write_blocking(i2c_default, addr, buf, 2, false);
	sleep_ms(250);
	uint8_t buf2[] = { 0x6B, 0x0 };
	i2c_write_blocking(i2c_default, addr, buf2, 2, false);
	sleep_ms(250);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
	// For this particular device, we send the device the register we want to read
	// first, then subsequently read from the device. The register is auto incrementing
	// so we don't need to keep sending the register we want, just the first.

	uint8_t buffer[6];

	// Start reading acceleration registers from register 0x3B for 6 bytes
	uint8_t val = 0x3B;
	i2c_write_blocking(i2c_default, addr, &val, 1,
			   true); // true to keep master control of bus
	i2c_read_blocking(i2c_default, addr, buffer, 6, false);

	for (int i = 0; i < 3; i++) {
		accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
	}

	// Now gyro data from reg 0x43 for 6 bytes
	// The register is auto incrementing on each read
	val = 0x43;
	i2c_write_blocking(i2c_default, addr, &val, 1, true);
	i2c_read_blocking(i2c_default, addr, buffer, 6,
			  false); // False - finished with bus

	for (int i = 0; i < 3; i++) {
		gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
		;
	}

	// Now temperature from reg 0x41 for 2 bytes
	// The register is auto incrementing on each read
	val = 0x41;
	i2c_write_blocking(i2c_default, addr, &val, 1, true);
	i2c_read_blocking(i2c_default, addr, buffer, 2,
			  false); // False - finished with bus

	*temp = buffer[0] << 8 | buffer[1];
}
#endif

int main()
{
	stdio_init_all();
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || \
	!defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/mpu6050_i2c example requires a board with I2C pins
	puts("Default I2C pins were not defined");
	return 0;
#else
	printf("Hello, MPU6050! Reading raw data from registers...\n");

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

	mpu6050_reset();

	int16_t acceleration[3], gyro[3], temp;
	float acceleration_f[3];
	float acceleration_mod;

	arc::sensors::MPU6050 mpu6050{ 0x68 };

	uint8_t scale = 0;
	while (1) {
		gpio_put(LED_PIN, 1);

		mpu6050.set_acc_scale(scale);

		scale = (scale + 1) % 4;

		mpu6050_read_raw(acceleration, gyro, &temp);

		acceleration_f[0] = acceleration[0] * mpu6050.m_acc_scale;
		acceleration_f[1] = acceleration[1] * mpu6050.m_acc_scale;
		acceleration_f[2] = acceleration[2] * mpu6050.m_acc_scale;
		acceleration_mod = sqrtf(acceleration_f[0] * acceleration_f[0] +
					 acceleration_f[1] * acceleration_f[1] +
					 acceleration_f[2] * acceleration_f[2]);

		printf("Acc. X = %f, Y = %f, Z = %f, |.|=%f\n",
		       acceleration_f[0], acceleration_f[1], acceleration_f[2],
		       acceleration_mod);
		printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1],
		       gyro[2]);
		printf("Temp. = %f\n", (temp / 340.0) + 36.53);

		sleep_ms(1000);

		gpio_put(LED_PIN, 0);
		sleep_ms(250);
	}
#endif
}
