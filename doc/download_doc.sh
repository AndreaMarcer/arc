#!/bin/bash

# PICO
wget -P ${ARC_DIR}/doc/pico/ https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf
wget -P ${ARC_DIR}/doc/pico/ https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
wget -P ${ARC_DIR}/doc/pico/ https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf
wget -P ${ARC_DIR}/doc/pico/ https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf
wget -P ${ARC_DIR}/doc/pico/ https://user-images.githubusercontent.com/759846/132600686-8341b469-a875-4e41-8ff7-2eb18e07e9da.png -O pico_gpio.png

# MPU6050
wget -P ${ARC_DIR}/doc/MPU6050/ https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
wget -P ${ARC_DIR}/doc/MPU6050/ https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf