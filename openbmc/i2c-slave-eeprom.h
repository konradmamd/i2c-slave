#ifndef I2C_SLAVE_EEPROM_H
#define I2C_SLAVE_EEPROM_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
typedef uint8_t __u8;
typedef uint16_t __u16;
typedef uint32_t __u32;
#endif

/*
 * This is a pure I2C drvier, however we set buffer size constants
 * based on the SMBus spec to allow SMBus applications to be built on
 * top of the interface this driver exposes.
 */
#define SMBUS_MAX_DATA                ( 255 )
#define SMBUS_MAX_BUFFER              ( SMBUS_MAX_DATA + 3 )  /* cmd + len + pec */
#define I2C_MAX_BUFFER                ( SMBUS_MAX_BUFFER )

#define IOC_GET_DATA_OVERHEAD         ( 1 )  /* wr_index */
#define IOC_GET_LAST_EVENT_OVERHEAD   ( 3 )  /* event + rd_index + wr_index */
#define IOC_MAX_BUFFER                ( I2C_MAX_BUFFER + IOC_GET_LAST_EVENT_OVERHEAD )  /* Absolute max buffer */

#define I2C_SLAVE_IOC_MAGIC          'i'
#define I2C_SLAVE_SET_DATA           _IOW( I2C_SLAVE_IOC_MAGIC, 0, __u8* )
#define I2C_SLAVE_GET_DATA           _IOR( I2C_SLAVE_IOC_MAGIC, 1, __u8* )
#define I2C_SLAVE_GET_LAST_EVENT     _IOR( I2C_SLAVE_IOC_MAGIC, 2, __u8* )
#define I2C_SLAVE_IOC_MAX            3

#define WRITE_DATA_EVENT             ( 1UL << 0 )  /* A master wrote to us */
#define READ_DATA_EVENT              ( 1UL << 1 )  /* A master read from us */
#define NO_DATA_EVENT                ( 1UL << 2 )  /* No data */

struct i2c_data {
    __u16 data_size;
    __u8  data[I2C_MAX_BUFFER];
};

struct i2c_event {
    __u8  event_type;
    __u16 rd_index;
    struct i2c_data wr_data;
};

#endif  /* I2C_SLAVE_EEPROM_H */
