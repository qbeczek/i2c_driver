import usb
import argparse
import time
from periphery import I2C


def main():
    parser = argparse.ArgumentParser(description="I2C over USB")
    parser.add_argument("-test", "--control_msg",
                        action="store_true", help="Send control messages")
    args = parser.parse_args()

    if args.control_msg:
        usb_read_test()
    else:
        i2c_test()


def usb_read_test():
    devs = usb.core.find(find_all=True)
    found = False
    for dev in devs:
        if dev.idVendor == 0x2312 and dev.idProduct == 0xec40:
            print("Device found!")
            found = True
            break
    if not found:
        raise Exception("Device not found!")

    dev.set_configuration()
    dev.ctrl_transfer(0x40, 0x5d, 1, 0, 0)
    time.sleep(2)
    dev.ctrl_transfer(0x40, 0x5e, 1, 0, 0)

    # w = dev.ctrl_transfer(0xc0, 0x5c, 0, 0, 64)
    # print("Received message: " + bytes(w).decode('utf-8'))


def i2c_test():
    # Open i2c-0 controller
    i2c = I2C("/dev/i2c-17")

    # Define the I2C address of the target device
    device_address = 0x50

    # Create a list of messages to send
    message_to_send = "TEST".encode('utf-8')
    messages = [
        I2C.Message(message_to_send),
        I2C.Message(data=[0x00, 0x00, 0x00, 0x00], read=True, flags=0),]

    # Transfer the list of messages to the target device
    i2c.transfer(device_address, messages)

    # Print sent and received data
    print("Sent data:", message_to_send)
    print("Received data:", messages[1].data)

    # Convert the received data (list of integers) to bytes
    received_data = bytes(messages[1].data)

    # Process only the relevant portion of the received data
    relevant_data = received_data[:len(message_to_send)]

    # Attempt to decode the received data as a UTF-8 string
    try:
        received_string = relevant_data.decode('utf-8')
        print("Received data as a string:", received_string)
    except UnicodeDecodeError:
        print("Received data could not be decoded as UTF-8 string")

    # Close the I2C connection
    i2c.close()


if __name__ == "__main__":
    main()
