#!/bin/bash

# Check the number of arguments
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <file to flash>"
    exit 1
fi

file_to_copy=$1

# Check if the file ends with .elf
if [[ ! "$file_to_copy" =~ \.elf$ ]]; then
    echo "Error: The file must have a .elf extension."
    exit 1
fi

# Check if the file exists
if [ ! -e "$file_to_copy" ]; then
    echo "Error: File '$file_to_copy' does not exist."
    exit 1
fi

openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program $1 verify reset exit"
