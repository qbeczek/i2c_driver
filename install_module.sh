#!/bin/bash

# Change these variables to match your project setup
module_name="i2c_driver.ko"

# ANSI escape code for red text
RED='\033[0;31m'

# Reset ANSI escape code
RESET='\033[0m'

display_error() {
    echo -e "${RED}Error: $1${RESET}"
    exit 1
}

# Step 1: Clean the project
make clean
if [ $? -ne 0 ]; then
    echodisplay_error "Error in 'make clean'"
fi

# Step 2: Build the project
make all
if [ $? -ne 0 ]; then
    display_error "Error in 'make all'"
fi

# Step 3: Check if the module is currently loaded
if lsmod | grep "i2c_driver"; then
    # Step 4: Remove the kernel module
    sudo rmmod "$module_name"
    if [ $? -ne 0 ]; then
        display_error "Error in 'sudo rmmod $module_name'"
    fi
fi

# Step 4: Insert the kernel module
sudo insmod "$module_name"
if [ $? -ne 0 ]; then
     display_error "Error in 'sudo insmod $module_name'"
fi

# Step 5: test with python script
# sudo python3 i2c_test.py
# if [ $? -ne 0 ]; then
#     display_error "Error in python testing"    
# fi

echo "Script completed successfully."
