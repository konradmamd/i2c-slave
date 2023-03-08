// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C slave mode EEPROM simulator
 *
 * Copyright (C) 2014 by Wolfram Sang, Sang Engineering <wsa@sang-engineering.com>
 * Copyright (C) 2014 by Renesas Electronics Corporation
 * Copyright (c) 2023 Advanced Micro Devices, Inc. All rights reserved.
 *
 * Because most slave IP cores can only detect one I2C slave address anyhow,
 * this driver does not support simulating EEPROM types which take more than
 * one address.
 */

/*
 * FIXME: What to do if only 8 bits of a 16 bit address are sent?
 * The ST-M24C64 sends only 0xff then. Needs verification with other
 * EEPROMs, though. We currently use the 8 bit as a valid address.
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

/* Original i2c-slave-eeprom includes */
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/types.h>

/* IOCTL includes */
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>

/* Poll includes */
#include <linux/wait.h>
#include <linux/poll.h>

#include "i2c-slave-eeprom.h"

#include <linux/delay.h>

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/

#define I2C_SLAVE_DEVICE_MAGIC(_len, _flags) ((_flags) | ((_len) - 1))

/*****************************************************************************/
/* Global variables                                                          */
/*****************************************************************************/

/* 
 * NOTE: Currently, only a single global slave device is supported.
 * Attempts to create a slave device if one already exists will fail.
 */
static bool slave_created = false;
static u16 rd_index = 0;
static u16 wr_index = 0;
static u8 rd_buffer[ I2C_MAX_BUFFER ] = { 0 };  /* Data we write to a master. */
static u8 wr_buffer[ I2C_MAX_BUFFER ] = { 0 };  /* Data which was written to us. */

static dev_t dev = 0;
static struct class *dev_class;
static struct cdev cdev;

static u8 last_event = NO_DATA_EVENT;
static DECLARE_WAIT_QUEUE_HEAD(slave_wait);

/*****************************************************************************/
/* Function declarations                                                     */
/*****************************************************************************/

/**
 * device_open() - Called when a process tries to open the character device file.
 * @inode: Pointer to a file on disk.
 * @filp: Pointer to an open file (associated with a file descriptor within
 *        a process). An inode can have any number of file structures
 *        associated with it.
 * 
 * Return: 0 on success, -ERROR on failure
 */
static int device_open(struct inode *inode, struct file *filp);

/**
 * device_release() - Called when a process closes the character device file.
 * @inode: Pointer to a file on disk.
 * @filp: Pointer to an open file (associated with a file descriptor within
 *        a process). An inode can have any number of file structures
 *        associated with it.
 * 
 * Return: 0 on success, -ERROR on failure
 */
static int device_release(struct inode *inode, struct file *filp);

/**
 * device_read() - Called when a process attempts to read from the device file.
 * @filp: Pointer to an open file device file
 * @buf: The buffer to fill with data
 * @len: The length of the buffer
 * @off: Our offset in the file
 * 
 * Return: Number of bytes read on success, -ERROR on failure
 */
static ssize_t device_read(struct file *filp, char __user *buf, size_t len, loff_t *off);

/**
 * device_write() - Called when a process writes to the character devices file.
 * 
 * @filp: Pointer to an open file device file
 * @buf: The buffer which was written
 * @len: Size of the buffer
 * @off: Offset in the file
 * 
 * Return: Number of bytes written on succes, -ERROR on failure
 */
static ssize_t device_write(struct file* filp, const char* buf, size_t len, loff_t* off);

/**
 * device_do_ioctl() - Called when a process executes an IOCTL.
 * 
 * @filp: Pointer to an open file device file
 * @cmd: The IOCTL command that was executed
 * @arg: The argument of the executed IOCTL. This can either be a long
 *       or a pointer to user-space memory.
 * 
 * Return: 0 on success, -ERROR on failure
 */
static long device_do_ioctl(struct file* file, unsigned int cmd, unsigned long arg);

/**
 * i2c_slave_cb() - This function will be called by the bus driver on interrupts.
 * @client: Pointer to the i2c_client struct for this device
 * @event: The event that was raised
 * @val: Pointer to byte value for this event
 * 
 * Return: 0
 */
static int i2c_slave_cb(struct i2c_client *client, enum i2c_slave_event event, u8 *val);

/**
 * i2c_slave_probe() - This function will be called when a new slave is created.
 * @client: Pointer to the i2c_client struct for this device
 * @id: Pointer to i2c_device_id struct for this device
 * 
 * Return: 0 on success, -ERROR on failure
 */
static int i2c_slave_probe(struct i2c_client *client, const struct i2c_device_id *id);

/**
 * i2c_slave_remove() - This function will be called when a slave device is removed.
 * @client: Pointer to the i2c_client struct for this device
 * 
 * Return: 0
 */
static int i2c_slave_remove(struct i2c_client *client);

/**
 * slave_poll() - This function will be called when a user polls the device.
 * @filp: Pointer to an open device file.
 * @wait: Opaque pointer to poll_table object.
 * 
 * Return: Non-zero bitmask when ready for reading and/or writing, 0 otherwise
 */
static unsigned int slave_poll(struct file *filp, poll_table *wait);

/*****************************************************************************/
/* Function implementations                                                  */
/*****************************************************************************/

/* Open the device file. */
static int device_open(struct inode *inode, struct file *filp)
{
    return 0;
}

/* Close the device file. */
static int device_release(struct inode *inode, struct file *filp)
{
    return 0;
}

/* Read from the device file. */
static ssize_t device_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    return 0;
}

/* Write to the device file. */
static ssize_t device_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    return len;
}

/* Execute an IOCTL command. */
static long device_do_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    /*
     * Extract the type and number bitfields, and don't decode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok(  )
     */
    if(_IOC_TYPE(cmd) != I2C_SLAVE_IOC_MAGIC) return -ENOTTY;
    if(_IOC_NR(cmd) > I2C_SLAVE_IOC_MAX) return -ENOTTY;

    if(!access_ok((void __user *)arg, _IOC_SIZE(cmd)))
        return -EFAULT;

    switch(cmd) {
    case I2C_SLAVE_SET_DATA:
    {
        /* The user wants to set I2C response data. */
        int i = 0;
        struct i2c_data rd_data = { 0 };

        if(copy_from_user((u8*)&rd_data, (u8*)arg, sizeof(rd_data)))
            return -EFAULT; /* Bad address */

        if(rd_data.data_size > I2C_MAX_BUFFER)
            return -ENOMEM; /* Out of memory */

        /* Set the rd_buffer */
        for(i = 0; i < rd_data.data_size; i++)
            rd_buffer[i] = rd_data.data[i];

        break;
    }

    case I2C_SLAVE_GET_DATA:
    {
        /* The user requests data that was written to the slave. */
        int i = 0;
        struct i2c_data wr_data = { 0 };

        if(wr_index < I2C_MAX_BUFFER)
            wr_data.data_size = wr_index;
        else
            wr_data.data_size = I2C_MAX_BUFFER;

        for(i = 0; i < wr_data.data_size; i++)
        {
            wr_data.data[i] = wr_buffer[i];
        }

        if(copy_to_user((u8*)arg, (u8*)&wr_data, sizeof(wr_data)))
            return -EFAULT;  /* Bad address */

        break;
    }

    case I2C_SLAVE_GET_LAST_EVENT:
    {
        /*
         * To avoid race conditions, we will return the event and ALL
         * of its related data/statistics as an `i2c_event` struct.
         */
        int i = 0;
        struct i2c_event event = { 0 };

        event.event_type = last_event;
        event.rd_index = rd_index;

        if(wr_index < I2C_MAX_BUFFER)
            event.wr_data.data_size = wr_index;
        else
            event.wr_data.data_size = I2C_MAX_BUFFER;
        
        for(i = 0; i < event.wr_data.data_size; i++)
        {
            event.wr_data.data[i] = wr_buffer[i];
        }

        if(copy_to_user((u8*)arg, (u8*)&event, sizeof(event)))
            return -EFAULT;

        last_event = NO_DATA_EVENT;  /* Clear last event. */
        break;
    }

    default:
        return -ENOTTY;  /* Invalid argument */
    }

    return 0;
}

/* Poll the device */
static unsigned int slave_poll(struct file *filp, poll_table *wait)
{
    poll_wait(filp, &slave_wait, wait);

    if((last_event & READ_DATA_EVENT) || (last_event & WRITE_DATA_EVENT))
        return POLLIN | POLLRDNORM;

    return 0;
}

/*
 * NOTE: The OpenBMC compiler does not allow '.field = value' syntax for
 * struct initialisation unless all fields are specified so we use
 * 'field: value' which is provided by a gcc extension!
 */
static struct file_operations fops = {
    read: device_read,
    write: device_write,
    open: device_open,
    release: device_release,
    unlocked_ioctl: device_do_ioctl,
    poll: slave_poll
};

/* Bus driver callback. */
static int i2c_slave_cb(struct i2c_client *client, enum i2c_slave_event event, u8 *val)
{
    switch(event) {
    case I2C_SLAVE_WRITE_REQUESTED:
        /* Another I2C master wants to write data to us. */
        last_event |= WRITE_DATA_EVENT;
        wr_index = 0;
        break;

    case I2C_SLAVE_WRITE_RECEIVED:
        /* Another I2C master wrote a byte to us. */
        if(wr_index < I2C_MAX_BUFFER)
            wr_buffer[wr_index++] = *val;

        break;

    case I2C_SLAVE_READ_REQUESTED:
        /*
        * Do not increment rd_index here, because we don't know if
        * this byte will be actually used. Read Linux I2C slave docs
        * for details.
        */
        rd_index = 0;
        last_event |= READ_DATA_EVENT;
        
        if(rd_index < I2C_MAX_BUFFER)
            *val = rd_buffer[rd_index];

        break;

    case I2C_SLAVE_READ_PROCESSED:
        /* The previous byte made it to the bus, get next one */
        rd_index++;

        if(rd_index < I2C_MAX_BUFFER)
            *val = rd_buffer[rd_index];

        break;

    case I2C_SLAVE_STOP:
        /* A stop condition was received. */
        if(last_event & READ_DATA_EVENT)
            rd_index++;  /* READ_PROCESSED does not get called again if we get a STOP */

        if((last_event & READ_DATA_EVENT) || (last_event & WRITE_DATA_EVENT))
            last_event &= ~NO_DATA_EVENT;

        wake_up_interruptible(&slave_wait);  /* Only wake up poll if the transaction is finished. */
        break;

    default:
        break;
    }

    return 0;
}

/* Create a new I2C slave */
static int i2c_slave_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;

    /* Only one slave permitted. */
    if(slave_created)
        return -EBUSY;  /* Device or resource busy */

    /* Register the slave device. */
    ret = i2c_slave_register(client, i2c_slave_cb);
    if(ret)
        return ret;

    /* Setup IOCTL */
    ret = alloc_chrdev_region(&dev, 0, 1, "i2c-simple-slave");
    if(ret)
        return ret;

    cdev_init(&cdev, &fops);

    /*Adding character device to the system*/
    if(cdev_add(&cdev, dev, 1))
        goto r_class;

    /* Creating struct class */
    if(IS_ERR(dev_class = class_create(THIS_MODULE, "i2c-simple-slave")))
        goto r_class;

    /* Creating device */
    if(IS_ERR(device_create(dev_class, NULL, dev, NULL, "i2c-slave-simple")))
        goto r_device;

    slave_created = true;
    return 0;

r_device:
    class_destroy(dev_class);
r_class:
    unregister_chrdev_region(dev, 1);
    return -ENODEV;  /* No such device */
}

/* Remove an I2C slave device */
static int i2c_slave_remove(struct i2c_client *client)
{
    /* Reset the slave created flag. */
    slave_created = false;

    /* Cleanup device data*/
    device_destroy(dev_class, dev);
    class_destroy(dev_class);
    cdev_del(&cdev);
    unregister_chrdev_region(dev, 1);

    /* Remove the slave */
    i2c_slave_unregister(client);

    return 0;
}

static const struct i2c_device_id i2c_slave_id[] = {
    { "i2c-slave-simple", I2C_SLAVE_DEVICE_MAGIC(2048 / 8,  0) },
    { }
};
MODULE_DEVICE_TABLE( i2c, i2c_slave_id );

static struct i2c_driver i2c_slave_driver = {
    .driver = {
        .name = "i2c-simple-slave",
    },
    .probe = i2c_slave_probe,
    .remove = i2c_slave_remove,
    .id_table = i2c_slave_id,
};
module_i2c_driver(i2c_slave_driver) ;

MODULE_AUTHOR("Wolfram Sang <wsa@sang-engineering.com>");
MODULE_DESCRIPTION("I2C slave mode EEPROM simulator");
MODULE_LICENSE("GPL v2");
