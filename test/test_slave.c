/**
 * @file test_slave.c
 * @author Konrad Mosoczy (konradm@amd.com)
 * @brief Application to test I2C slave functionality on OpenBMC/Linux
 * @date 2023-03-06
 * 
 * Copyright (c) 2023-present Advanced Micro Devices, Inc. All rights reserved.
 * 
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>

#include "test_utils.h"
#include "i2c-slave-eeprom.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/

#define INVALID_FD            ( -1 )
#define LINUX_ERROR           ( -1 )
#define LINUX_OK              ( 0  )

#define DEVICE_NAME           "i2c-slave-simple"
#define DEVICE_FILE           "/dev/" DEVICE_NAME

#define NEW_DEVICE            DEVICE_NAME " 0x10%02hhX"
#define DEL_DEVICE            "0x10%02hhX"
#define NEW_DEVICE_SYSFS_STR  "/sys/bus/i2c/devices/i2c-%d/new_device"
#define DEL_DEVICE_SYSFS_STR  "/sys/bus/i2c/devices/i2c-%d/delete_device"
#define DEVICE_SYSFS_STR_LEN  ( 50 )

#define POLL_FOREVER          ( -1 )

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/**
 * @enum USER_CMD
 * @brief List of test commands
 */
typedef enum USER_CMD
{
    USER_CMD_QUIT = 0,

    USER_CMD_HELP,
    USER_CMD_SET_DATA,
    USER_CMD_GET_DATA,
    USER_CMD_POLL,
    USER_CMD_OPEN_DEVICE,
    USER_CMD_NEW_DEVICE,
    USER_CMD_DELETE_DEVICE,

    MAX_USER_CMD

} USER_CMD;

/*****************************************************************************/
/* Local variables                                                           */
/*****************************************************************************/

static struct i2c_data  xTxData = { 0 };
static struct i2c_event xRxData = { 0 };
static bool bQuit               = false;
static bool bStopPolling        = false;
static int iDevice              = INVALID_FD;

/*****************************************************************************/
/* Function Declarations                                                     */
/*****************************************************************************/

/**
 * @brief Displays user commands
 *
 * @returns None
 */
static void vDisplayCommands( void );

/**
 * @brief Open the i2c-slave-simple sysfs interface.
 *
 * @returns None
 */
static void vOpenDevice( void );

/**
 * @brief Add a new slave device on the specified bus via its sysfs interface.
 * 
 * @param iBusNum The bus number to use
 * @param ucAddr Slave address
 *
 * @returns None
 */
static void vNewSlaveDevice( int iBusNum, uint8_t ucAddr );

/**
 * @brief Remove a slave device from the specified bus via its sysfs interface.
 * 
 * @param iBusNum The bus number to use
 * @param ucAddr Slave address
 *
 * @returns None
 */
static void vDeleteSlaveDevice( int iBusNum, uint8_t ucAddr );

/**
 * @brief Poll the I2C slave for any events.
 * 
 * @returns None
 */
static void vPollSlaveLoop( void );

/**
 * @brief Set the slave I2C response data.
 * 
 * @returns None
 */
static void vSetSlaveData( void );

/**
 * @brief Get the slave I2C data written by a master. This does not check
 *   if a transaction was completed fully or if there even is any data. Also,
 *   the last_event on the driver side does not get cleared.
 * 
 * @returns None
 */
static void vGetSlaveData( void );

/**
 * @brief Get the last event/events received by the I2C slave.
 * 
 * @returns None
 * 
 */
static void vGetSlaveEvent( void );

/**
 * @brief Interrupt handler to stop polling when Ctrl-C is pressed.
 * 
 * @param iSig The received signal - only handles SIGINT.
 * 
 * @returns None
 */
static void vStopPolling( int iSig );

/*****************************************************************************/
/* Function Implementations                                                  */
/*****************************************************************************/

/**
 * @brief Print commands
 */
static void vDisplayCommands( void )
{
    NEW_LINE;
    PRINT_ENUM( USER_CMD_QUIT );
    PRINT_ENUM( USER_CMD_HELP );
    PRINT_ENUM( USER_CMD_SET_DATA );
    PRINT_ENUM( USER_CMD_GET_DATA );
    PRINT_ENUM( USER_CMD_POLL );
    PRINT_ENUM( USER_CMD_OPEN_DEVICE );
    PRINT_ENUM( USER_CMD_NEW_DEVICE );
    PRINT_ENUM( USER_CMD_DELETE_DEVICE );
    NEW_LINE;
}

/**
 * @brief Open the i2c slave sysfs interface.
 */
static void vOpenDevice( void )
{
    if( INVALID_FD == iDevice )
    {
        iDevice = open( DEVICE_FILE, O_RDWR | O_NONBLOCK );

        if( INVALID_FD != iDevice )
        {
            SMBUS_PRINTF( "Opened %s sysfs interface\r\n", DEVICE_FILE );
        }
        else
        {
            SMBUS_PRINTF_ERR( "Coult not open %s sysfs interface\r\n", DEVICE_FILE );
        }
    }
    else
    {
        SMBUS_PRINTF_ERR( "Device already open!\r\n" );
    }
}

/**
 * @brief Create a new slave device.
 */
static void vNewSlaveDevice( int iBusNum, uint8_t ucAddr )
{
    char pcNewDeviceSysfs[ DEVICE_SYSFS_STR_LEN ] = { 0 };
    snprintf( pcNewDeviceSysfs, DEVICE_SYSFS_STR_LEN - 1, NEW_DEVICE_SYSFS_STR, iBusNum );

    int iBusFd = open( pcNewDeviceSysfs, O_WRONLY );

    if( INVALID_FD != iBusFd )
    {
        char pcNewDevice[ DEVICE_SYSFS_STR_LEN ] = { 0 };
        snprintf( pcNewDevice, DEVICE_SYSFS_STR_LEN - 1, NEW_DEVICE, ucAddr );

        /* eg: echo i2c-slave-simple 0x1064 > /sys/bus/i2c/devices/i2c-10/new_device */
        if( LINUX_ERROR != write( iBusFd, &pcNewDevice, strlen( pcNewDevice ) ) )
        {
            SMBUS_PRINTF( "Created slave device on bus %d at address 0x%02hhX\r\n", iBusNum, ucAddr );
        }
        else
        {
            SMBUS_PRINTF_ERR( "Could not create slave device!\r\n" );
        }

        close( iBusFd );
    }
    else
    {
        SMBUS_PRINTF_ERR( "Could not open bus %d sysfs interface!\r\n", iBusNum );
    }
}

/**
 * @brief Remove an existing slave device.
 */
static void vDeleteSlaveDevice( int iBusNum, uint8_t ucAddr )
{
    char pcDelDeviceSysfs[ DEVICE_SYSFS_STR_LEN ] = { 0 };
    snprintf( pcDelDeviceSysfs, DEVICE_SYSFS_STR_LEN - 1, DEL_DEVICE_SYSFS_STR, iBusNum );

    int iBusFd = open( pcDelDeviceSysfs, O_WRONLY );

    if( INVALID_FD != iBusFd )
    {
        char pcDelDevice[ DEVICE_SYSFS_STR_LEN ] = { 0 };
        snprintf( pcDelDevice, DEVICE_SYSFS_STR_LEN - 1, DEL_DEVICE, ucAddr );

        /* Close the current device before deleting. */
        if( INVALID_FD != iDevice )
        {
            SMBUS_PRINTF( "Closing device file...\r\n" );
            close( iDevice );
            iDevice = INVALID_FD;
        }

        /* eg: echo i2c-slave-simple 0x1064 > /sys/bus/i2c/devices/i2c-10/new_device */
        if( LINUX_ERROR != write( iBusFd, &pcDelDevice, strlen( pcDelDevice ) ) )
        {
            SMBUS_PRINTF( "Removed slave device on bus %d at address 0x%02hhX\r\n", iBusNum, ucAddr );
        }
        else
        {
            SMBUS_PRINTF_ERR( "Could not remove slave device!\r\n" );
        }

        close( iBusFd );
    }
    else
    {
        SMBUS_PRINTF_ERR( "Could not open bus %d sysfs interface!\r\n", iBusNum );
    }
}

/**
 * @brief Stop the slave poll loop
 */
static void vStopPolling( int iSig )
{
    switch( iSig )
    {
        case SIGINT:
        {
            bStopPolling = true;
            break;
        }

        default:
        {
            break;
        }
    }
}

/**
 * @brief Poll the i2c slave for events.
 */
static void vPollSlaveLoop( void )
{
    if( INVALID_FD != iDevice )
    {
        int iRet = -1;

        struct pollfd xPoll = { 0 };
        xPoll.fd = iDevice;
        xPoll.events = POLLIN | POLLRDNORM;

        /* Capture Ctrl-C signals. */
        signal( SIGINT, vStopPolling );

        while( !bStopPolling )
        {
            SMBUS_PRINTF( "Starting poll. Press Ctrl-C to stop.\r\n" );
            iRet = poll( &xPoll, ( unsigned long )1, POLL_FOREVER );

            if( iRet < 0 )
            {
                SMBUS_PRINTF_ERR( "Poll timeout.\r\n" );
                break;
            }

            if( ( xPoll.revents & POLLIN ) == POLLIN )
            {
                vGetSlaveEvent();
            }
        }

        /* Remove signal handler. */
        bStopPolling = false;
        signal( SIGINT, SIG_DFL );
    }
    else
    {
        SMBUS_PRINTF_ERR( "Invalid device file!\r\n" );
    }
}

/**
 * @brief Set the slave response data.
 */
static void vSetSlaveData( void )
{
    if( INVALID_FD != iDevice )
    {
        int i = 0;
        int iRet = 0;

        SMBUS_PRINTF( "Setting response data.\r\n" );

        /* Set the response buffer. */
        for( i = 0; i < I2C_MAX_BUFFER; i++ )
        {
            xTxData.data[ i ] = i;
        }

        xTxData.data_size = I2C_MAX_BUFFER;

        SMBUS_PRINTF( "Writing value to driver\r\n" );
        iRet = ioctl( iDevice, I2C_SLAVE_SET_DATA, ( uint8_t* )&xTxData );
        SMBUS_PRINTF( "Slave response buffer set with return value %d\r\n", iRet );
    }
    else
    {
        SMBUS_PRINTF_ERR( "Invalid device file\r\n" );
    }
}

/**
 * @brief Get data written to the slave.
 */
static void vGetSlaveData( void )
{
    if( INVALID_FD != iDevice )
    {
        int iRet = 0;
        SMBUS_PRINTF( "Reading value from driver\r\n" );
        iRet = ioctl( iDevice, I2C_SLAVE_GET_DATA, ( uint8_t* )&xRxData.wr_data );
        SMBUS_PRINTF( "IOCTL complete - got %d byte(s) with return value %d\r\n", xRxData.wr_data.data_size, iRet );
        vDisplayBuffer( &xRxData.wr_data.data[ 0 ], xRxData.wr_data.data_size, NULL );
    }
    else
    {
        SMBUS_PRINTF_ERR( "Invalid device file\r\n" );
    }
}

/**
 * @brief Get the last slave event.
 */
static void vGetSlaveEvent( void )
{
    if( INVALID_FD != iDevice )
    {
        int iRet = 0;
        
        SMBUS_PRINTF( "Reading value from driver\r\n" );
        iRet = ioctl( iDevice, I2C_SLAVE_GET_LAST_EVENT, ( uint8_t* )&xRxData );

        SMBUS_PRINTF(
            "IOCTL complete - got %d byte(s) with return value %d\r\n",
            xRxData.wr_data.data_size, iRet
        );

        vDisplayBuffer( &xRxData.wr_data.data[ 0 ], xRxData.wr_data.data_size, NULL );
    }
    else
    {
        SMBUS_PRINTF_ERR( "Invalid device file\r\n" );
    }
}

/**
 * @brief Run the main slave loop.
 */
int main()
{
    uint8_t ucInput = USER_CMD_QUIT;

    /* Try to open the device if it exists. The user can also do this later with a command. */
    if( LINUX_OK == access( DEVICE_FILE, F_OK ) )
    {
        SMBUS_PRINTF( "Found existing device, opening...\r\n" );
        vOpenDevice();
    }
    else
    {
        SMBUS_PRINTF( "Could not find existing device. Create a new one with the appropriate command.\r\n" );
    }

    vDisplayCommands();

    do
    {
        ucInput = ucInputByte( "Enter command", false, false );

        switch( ucInput )
        {
            case USER_CMD_SET_DATA:
            {
                vSetSlaveData();
                break;
            }

            case USER_CMD_GET_DATA:
            {
                vGetSlaveData();
                break;
            }

            case USER_CMD_POLL:
            {
                vPollSlaveLoop();
                break;
            }

            case USER_CMD_HELP:
            {
                vDisplayCommands();
                break;
            }

            case USER_CMD_OPEN_DEVICE:
            {
                vOpenDevice();
                break;
            }

            case USER_CMD_NEW_DEVICE:
            {
                uint8_t ucBusNum = ucInputByte( "Enter bus number", false, false );
                uint8_t ucAddr = ucInputByte( "Enter slave address", true, false );
                vNewSlaveDevice( ( int )ucBusNum, ucAddr );
                break;
            }

            case USER_CMD_DELETE_DEVICE:
            {
                uint8_t ucBusNum = ucInputByte( "Enter bus number", false, false );
                uint8_t ucAddr = ucInputByte( "Enter slave address", true, false );
                vDeleteSlaveDevice( ( int )ucBusNum, ucAddr );
                break;
            }

            case USER_CMD_QUIT:
            {
                bQuit = true;
                break;
            }

            default:
            {
                SMBUS_PRINTF_ERR( "Invalid command (%d)\r\n", ucInput );
                vDisplayCommands();
                break;
            }
        }

    } while( false == bQuit );

    /* Close the device file if it was opened. */
    if( INVALID_FD != iDevice )
    {
        SMBUS_PRINTF( "Closing device file...\r\n" );
        close( iDevice );
    }
}
