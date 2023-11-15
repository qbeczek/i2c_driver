from periphery import I2C

# Open i2c-0 controller
i2c = I2C("/dev/i2c-19")

# Define the I2C address of the target device
device_address = 0x50

# Create an instance of the Message class for a read operation
read_message = I2C.Message(data=[0x00], read=True)

# Transfer the read message via I2C
i2c.transfer(device_address, [read_message])

# Convert the received data (list of integers) to bytes
received_data = bytes(read_message.data)

# Attempt to decode the received data as a UTF-8 string
try:
    received_string = received_data.decode('utf-8')
    print("Received data as a string: ", received_string)
except UnicodeDecodeError:
    print("Received data could not be decoded as UTF-8 string")

# Close the I2C connection
i2c.close()
