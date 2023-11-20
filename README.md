# Custom Linux I2C Driver for Zephyr OS Firmware

This repository contains a Linux I2C driver designed to communicate with custom firmware running on a Zephyr OS device. The driver facilitates communication by exchanging messages with the device using the I2C protocol and sends frames to the device via USB. The driver module can be compiled using the provided Makefile.

## Requirements

- Linux operating system
- Custom firmware running on a Zephyr OS device with the following parameters:
  - Vendor ID: 0x2312
  - Product ID: 0xec40 (modifiable)

## Compilation

To compile the I2C driver module, follow these steps:

1. Clean the project: `make clean`
2. Build the project: `make`

## Usage

1. Check if the module is currently loaded: `lsmod | grep i2c_driver`
2. Insert the kernel module: `sudo insmod i2c_driver.ko`
3. Alternatively, you can use the provided script to install the driver: `sudo ./install_module`

### Removing the Module

To remove the module, use the following command:

```bash
sudo rmmod i2c_driver.ko
```

### Communication with Zephyr OS Firmware

The driver facilitates communication with the Zephyr OS firmware via I2C messages and sends frames to the device via USB. The communication methods include:

    * I2C Messages: Messages can be exchanged with the Zephyr OS firmware using the I2C protocol.

    * USB Frames: Frames are sent to the device via USB, enabling data exchange between the driver and the Zephyr OS device.


### Installation Verification

After installation, run the following command to check the messages in the kernel log:

```bash
sudo dmesg
```
You should see messages similar to the following:
```bash
[ 4321.941960] usbcore: registered new interface driver i2c_over_usb
````

And then plug the usb deivce and you can see information about device in the kernel log:
```bash
[ 4341.563761] usb 3-1: new full-speed USB device number 16 using xhci_hcd
[ 4341.714367] usb 3-1: New USB device found, idVendor=2312, idProduct=ec40, bcdDevice= 3.05
[ 4341.714378] usb 3-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 4341.714382] usb 3-1: Product: USB Non Standard Class
[ 4341.714385] usb 3-1: Manufacturer: ZEPHYR
[ 4341.714388] usb 3-1: SerialNumber: 33395114005E0057
[ 4341.717610] i2c_over_usb 3-1:1.0: probing usb device
[ 4341.717612] i2c_over_usb 3-1:1.0: Device VID: 2312, PID: ec40
[ 4341.717613] i2c_over_usb 3-1:1.0: Devnum: 16, Product: USB Non Standard Class
[ 4341.717613] i2c_over_usb 3-1:1.0: version 3.05 found at bus 003 address 016
[ 4341.717615]  (null): CMD: 93, CMD_I2C_FIND_DEVICE
[ 4341.717616] Sending message: FIND
[ 4341.717946] i2c i2c-19: connected i2c_over_usbdevice
```