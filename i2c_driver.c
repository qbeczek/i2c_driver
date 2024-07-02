/* @brief
 *   i2c over USB driver
 */

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/usb.h>
// #include <stddef.h>

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("KUBA");
MODULE_DESCRIPTION("Driver for stm32 i2c device");

#define VENDOR_ID 0x2312
#define PRODUCT_ID 0xec40
#undef CTRL_MSG_TEST

#define I2C_READ 1
#define I2C_WRITE 0
/* commands via USB, must match command ids in the firmware */
#define CMD_ECHO 0
#define CMD_GET_FUNC 1
#define CMD_SET_DELAY 2
#define CMD_GET_STATUS 3

#define CMD_I2C_IO 4
#define CMD_I2C_IO_BEGIN (1 << 0)
#define CMD_I2C_IO_END (1 << 1)

#define I2C_FLAG_FIND_DEVICE 8 /* your new flag 1 */
#define I2C_FLAG_READ_BYTE 9   /* your new flag 2 */
#define I2C_FLAG_3 10          /* your new flag 3 */
#define I2C_FLAG_4 0x80000     /* your new flag 4 */
#define I2C_FLAG_5 0x100000    /* your new flag 5 */

#define CMD_I2C_FIND_DEVICE 0x5d
#define CMD_I2C_READ_BYTE 0x5e

#define CMD_READ_FROM_BUFFER 0x5c
#define CMD_WRITE_TO_BUFFER 0x5b

struct i2c_command {
    uint8_t operation;  // 1 = write, 0 = read
    uint8_t addr;       // Adres urządzenia I2C
    uint8_t reg;        // Adres rejestru
    uint8_t length;     // Długość danych
    uint8_t data[64];   // Dane do zapisu lub bufor do odczytu
};

/* i2c bit delay, default is 10us -> 100kHz */
static int delay = 10;
module_param(delay, int, 0);
MODULE_PARM_DESC(delay,
                 "bit delay in microseconds, "
                 "e.g. 10 for 100kHz (default is 100kHz)");

// static int usb_read(struct i2c_adapter *adapter, uint8_t cmd,
//                     struct i2c_command *i2c_cmd, size_t cmd_len, uint8_t
//                     *buf, size_t buf_len);

static int usb_read(struct i2c_adapter *adapter, int request, int value,
                    int index, void *data, int size);
static int usb_write(struct i2c_adapter *adapter, uint8_t cmd,
                     struct i2c_command *i2c_cmd, size_t cmd_len);

void print_buffer(__u8 *buf, size_t buf_size) {
    for (size_t i = 0; i < buf_size; i++) {
        printk("buf[%zu] = %c\n", i, buf[i]);
    }
}
static int usb_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
                    int num_msgs) {
    int ret, i;
    struct i2c_command cmd;
    struct i2c_msg *msg;

    dev_info(&adapter->dev, "master xfer %d messages:\n", num_msgs);

    for (i = 0; i < num_msgs; i++) {
        msg = &msgs[i];

        cmd.operation = (msg->flags & I2C_M_RD) ? 0 : 1;
        cmd.addr = msg->addr;
        cmd.reg = msg->buf[0];
        cmd.length = 8;
        memcpy(cmd.data, &msg->buf[1], cmd.length);

        if (cmd.operation == 0) {
            // Odczyt danych
            dev_info(&adapter->dev, "Reading %d bytes from 0x%02x\n",
                     cmd.length, cmd.addr);
            // ret = usb_read(adapter, CMD_READ_FROM_BUFFER, &cmd, sizeof(cmd),
            //                msg->buf, msg->len);
            ret = usb_read(adapter, CMD_READ_FROM_BUFFER, msg->flags, msg->addr,
                           msg->buf, msg->len);
            if (ret < 0) {
                dev_err(&adapter->dev, "failure reading data\n");
                return -EREMOTEIO;
            }
        } else {
            // Zapis danych
            dev_info(&adapter->dev, "Writing %d bytes to 0x%02x\n", cmd.length,
                     cmd.addr);

            printk("Register: %x", cmd.reg);
            ret = usb_write(adapter, CMD_WRITE_TO_BUFFER, &cmd, sizeof(cmd));
            if (ret < 0) {
                dev_err(&adapter->dev, "failure writing data\n");
                return -EREMOTEIO;
            }
        }
    }

    return num_msgs;
}

static uint32_t usb_func(struct i2c_adapter *adapter) {
    return (I2C_FUNC_I2C | I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE |
            I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
            I2C_FUNC_SMBUS_BLOCK_DATA);
}

/* This is the actual algorithm we define */
static const struct i2c_algorithm usb_algorithm = {
    .master_xfer = usb_xfer,
    .functionality = usb_func,
};

/*-----------------------------------------------------------------------------


-------------------------------------------------------------------------------*/
static struct usb_device *usb_dev;

static struct usb_device_id i2c_over_usb_id_table[] = {
    {USB_DEVICE(VENDOR_ID, PRODUCT_ID)},
    {},
};

MODULE_DEVICE_TABLE(usb, i2c_over_usb_id_table);

/* Structure to hold all of our device specific stuff */
struct i2c_over_usb {
    struct usb_device *usb_dev;      /* the usb device for this device */
    struct usb_interface *interface; /* the interface for this device */
    struct i2c_adapter adapter;      /* i2c related things */
};

// static int usb_read(struct i2c_adapter *adapter, int request, int value,
//                     int index, void *data, int size) {
//     struct i2c_over_usb *dev = (struct i2c_over_usb *)adapter->algo_data;
//     char text[32];
//     int status;
//     int transferred;
//     uint8_t *buffer = kzalloc(64, GFP_KERNEL);

//     if (!buffer) return -ENOMEM;

//     /* do control transfer */
//     printk("Buffer size: %zu, Transferred before: %zu\n", sizeof(buffer),
//            transferred);
//     status = usb_bulk_msg(dev->usb_dev, usb_rcvbulkpipe(dev->usb_dev, 0x81),
//                           buffer, 64, &transferred, 10000);
//     printk("Transferred after: %zu\n", transferred);

//     // status = usb_control_msg(dev->usb_dev, usb_rcvctrlpipe(dev->usb_dev,
//     // 0),
//     //                          request, 0xc0, value, index, buffer, size,
//     //                          2000);

//     if (status < 0)
//         printk("i2c_driver: control usb_read error: %d", status);
//     else
//         printk("i2c_driver: data received: %s", buffer);

//     memcpy(data, buffer, size);
//     kfree(buffer);

//     return status;
// }

static int usb_read(struct i2c_adapter *adapter, int request, int value,
                    int index, void *data, int size) {
    printk("USB_READ_______________________");
    int ret;
    int transferred;

    struct i2c_over_usb *dev = (struct i2c_over_usb *)adapter->algo_data;
    uint8_t *buffer = kzalloc(64, GFP_KERNEL);
    // ret = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, 1),
    //                    (uint8_t *)i2c_cmd, sizeof(i2c_cmd), NULL, 1000);
    if (ret < 0) {
        dev_err(&adapter->dev, "USB bulk msg send failed: %d\n", ret);
        return ret;
    }
    ret = usb_bulk_msg(dev->usb_dev, usb_rcvbulkpipe(dev->usb_dev, 0x81),
                       buffer, 64, &transferred, 1000);
    printk("Transferred after: %zu\n", transferred);

    // ret = usb_bulk_msg(dev->usb_dev, usb_rcvbulkpipe(dev->usb_dev, 0x81),
    // buf,
    //                    buf_len, NULL, 1000);
    if (ret < 0) {
        dev_err(&adapter->dev, "USB bulk msg receive failed: %d\n", ret);
        return ret;
    }

    memcpy(data, buffer, size);
    kfree(buffer);

    return 0;
}

// static int usb_write(struct i2c_adapter *adapter, int request, int value,
//                      int index, void *data, int size) {
//     struct i2c_over_usb *dev = (struct i2c_over_usb *)adapter->algo_data;
//     int transferred;
//     long val;

//     int return_val;

//     return_val = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, 1),
//                               (uint8_t *)data, sizeof(data), &transferred,
//                               100);
//     printk("Trasnferred %d", transferred);
//     // Konwersja void* data na uint8_t* i wydrukowanie wszystkich elementów
//     uint8_t *byte_data = (uint8_t *)data;
//     struct i2c_command *i2c_msg = (struct i2c_command *)data;

//     for (int i = 0; i < 8; i++) {  // Poprawiono warunek z i > 8 na i < size
//         printk("%x ", byte_data[i]);
//     }

//     printk("\n");
//     printk("Sending I2C message: addr=0x%x, reg=0x%x", i2c_msg->device_addr,
//            i2c_msg->reg_addr);
//     // return_val =
//     //     usb_control_msg(dev->usb_dev, usb_sndctrlpipe(dev->usb_dev, 0),
//     //     request,
//     //                     0x40, value, 0, (uint8_t *)data, sizeof(data),
//     100); if (return_val < 0) {
//         printk("Control usb_write error: %d", return_val);
//     }

//     return return_val;
// }

static int usb_write(struct i2c_adapter *adapter, uint8_t cmd,
                     struct i2c_command *i2c_cmd, size_t cmd_len) {
    printk("USB_WRITE_______________________");

    struct i2c_over_usb *dev = (struct i2c_over_usb *)adapter->algo_data;
    int transferred;
    dev_info(&adapter->dev, "Sending USB bulk message:\n");
    dev_info(&adapter->dev, "cmd_len: %zu\n", cmd_len);
    dev_info(&adapter->dev,
             "i2c_cmd operation: %d, addr: 0x%02x, reg: 0x%02x, length: %d\n",
             i2c_cmd->operation, i2c_cmd->addr, i2c_cmd->reg, i2c_cmd->length);

    int ret = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, 1),
                           (uint8_t *)i2c_cmd, sizeof(i2c_cmd), NULL, 1000);
    // int ret = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, 1), 0,
    // 0,
    //    &transferred, 100);
    printk("ret: %d\n", ret);
    if (ret == 0) {
        return 0;
    }
    if (ret == -EAGAIN) {
        dev_warn(&adapter->dev,
                 "USB bulk msg send failed with EAGAIN, retrying...\n");
    } else {
        dev_err(&adapter->dev, "USB bulk msg send failed: %d\n", ret);
        return ret;
    }
    return ret;
}

static void i2c_over_usb_free(struct i2c_over_usb *dev) {
    usb_put_dev(dev->usb_dev);
    kfree(dev);
}

static int i2c_over_usb_probe(struct usb_interface *interface,
                              const struct usb_device_id *id) {
    struct i2c_over_usb *dev;
    int retval = 0;
    u16 version;

    dev_info(&interface->dev, "probing usb device\n");

    /* allocate memory for our device state and initialize it */
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (dev == NULL) {
        dev_err(&interface->dev, "Out of memory\n");
        retval = -ENOMEM;
        if (dev) i2c_over_usb_free(dev);

        return retval;
    }

    dev->usb_dev = usb_get_dev(interface_to_usbdev(interface));
    dev->interface = interface;

    dev_info(&interface->dev, "Device VID: %x, PID: %x", id->idVendor,
             id->idProduct);
    dev_info(&interface->dev, "Devnum: %d, Product: %s", dev->usb_dev->devnum,
             dev->usb_dev->product);
    /* save our data pointer in this interface device */
    usb_set_intfdata(interface, dev);

    version = le16_to_cpu(dev->usb_dev->descriptor.bcdDevice);
    dev_info(&interface->dev,
             "version %x.%02x found at bus %03d address %03d\n", version >> 8,
             version & 0xff, dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

    /* setup i2c adapter description */
    dev->adapter.owner = THIS_MODULE;
    dev->adapter.class = I2C_CLASS_HWMON;
    dev->adapter.algo = &usb_algorithm;
    dev->adapter.algo_data = dev;

    snprintf(dev->adapter.name, I2C_NAME_SIZE,
             "i2c-over-usb at bus %03d device %03d", dev->usb_dev->bus->busnum,
             dev->usb_dev->devnum);

    dev->adapter.dev.parent = &dev->interface->dev;

    /* and finally attach to i2c layer */
    i2c_add_adapter(&dev->adapter);

    // int result = usb_write(&dev->adapter, 0x5b, cpu_to_le16(10), 0, "TEST",
    // 4);
    int result = 0;

    if (result < 0) {
        dev_err(&dev->adapter.dev, "failure test communication %d\n ", result);
        retval = -EIO;
        if (dev) i2c_over_usb_free(dev);
        return retval;
    }
    usb_clear_halt(dev->usb_dev, usb_rcvbulkpipe(dev->usb_dev, 1));

    // TODO: zobaczyć te strukturę
    // usb_write(&dev->adapter, 0, 0, 0, 0, sizeof(0));

    /* inform user about successful attachment to i2c layer */
    dev_info(&dev->adapter.dev, "connected i2c_over_usbdevice\n");

    return 0;
}

static void i2c_over_usb_disconnect(struct usb_interface *interface) {
    struct i2c_over_usb *dev = usb_get_intfdata(interface);

    i2c_del_adapter(&dev->adapter);
    usb_set_intfdata(interface, NULL);
    i2c_over_usb_free(dev);

    dev_info(&interface->dev, "disconnected\n");
}

static struct usb_driver i2c_over_usb_driver = {
    .name = "i2c_over_usb",
    .probe = i2c_over_usb_probe,
    .disconnect = i2c_over_usb_disconnect,
    .id_table = i2c_over_usb_id_table,
};

static int __init usb_i2c_over_usb_init(void) {
    /* register this driver with the USB subsystem */
    return usb_register(&i2c_over_usb_driver);
}

static void __exit usb_i2c_over_usb_exit(void) {
    /* deregister this driver with the USB subsystem */
    usb_deregister(&i2c_over_usb_driver);
}

module_init(usb_i2c_over_usb_init);
module_exit(usb_i2c_over_usb_exit);
