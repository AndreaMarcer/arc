#!/bin/bash

# Check the number of arguments
if [ "$#" -eq 0 ]; then
    file_to_flash=${ARC_DIR}/build/arc/arc.elf
else if [ "$#" -eq 1 ]; then
    file_to_flash=$1
else 
    echo "Usage: $0 <file to flash>"
    exit 1
fi
fi

# Check if the file ends with .elf
if [[ ! "$file_to_flash" =~ \.elf$ ]]; then
    echo "Error: The file must have a .elf extension."
    exit 1
fi

# Check if the file exists
if [ ! -e "$file_to_flash" ]; then
    echo "Error: File '$file_to_flash' does not exist."
    exit 1
fi

echo "FLASHING: ${file_to_flash}"

openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program ${file_to_flash} verify reset exit"
