#!/bin/bash

# Check the number of arguments
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <file to flash>"
    exit 1
fi

file_to_copy=$1

# Check if the file exists
if [ ! -e "$file_to_copy" ]; then
    echo "Error: File '$file_to_copy' does not exist."
    exit 1
fi

# Check if the file ends with .uf2
if [[ ! "$file_to_copy" =~ \.uf2$ ]]; then
    echo "Error: The file must have a .uf2 extension."
    exit 1
fi

# List devices matching /dev/sd*1
devices=(/dev/sd*1)
num_devices=${#devices[@]}

# Check if any devices were found
if [ $num_devices -eq 0 ]; then
    echo "No devices found matching /dev/sd*1."
    exit 1
fi

# Display the list of devices
echo "Available devices:"
for ((i=0; i<$num_devices; i++)); do
    echo "[$i] ${devices[$i]}"
done

# Prompt user to choose a device
read -p "Enter the number of the device you want to select: " choice

# Validate user input
if ! [[ "$choice" =~ ^[0-9]+$ ]]; then
    echo "Invalid input. Please enter a number."
    exit 1
fi

# Check if the chosen number is within the valid range
if [ $choice -ge 0 ] && [ $choice -lt $num_devices ]; then
    selected_device=${devices[$choice]}
    echo "You selected: $selected_device"

    # Check if the device is already mounted
    if mountpoint -q /mnt/pico; then
        echo "Device is already mounted to /mnt/pico."

        # Copy the specified file to /mnt/pico
        sudo cp "$file_to_copy" /mnt/pico
        if [ $? -eq 0 ]; then
            echo "File '$file_to_copy' successfully copied to /mnt/pico."
        else
            echo "Error: Failed to copy the file to /mnt/pico."
        fi

    else
        # Create /mnt/pico directory if it doesn't exist
        sudo mkdir -p /mnt/pico

        # Mount the selected device to /mnt/pico
        sudo mount $selected_device /mnt/pico
        if [ $? -eq 0 ]; then
            echo "Device successfully mounted to /mnt/pico."

            # Copy the specified file to /mnt/pico
            sudo cp "$file_to_copy" /mnt/pico
            if [ $? -eq 0 ]; then
                echo "File '$file_to_copy' successfully copied to /mnt/pico."
            else
                echo "Error: Failed to copy the file to /mnt/pico."
            fi

        else
            echo "Error: Failed to mount the device. Check if it's already mounted or try again."
            exit 1
        fi
    fi

else
    echo "Invalid selection. Please choose a number within the range."
    exit 1
fi
