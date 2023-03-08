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

#define INVALID_FD          ( -1  )
#define DEVICE_FILE         ( "/dev/i2c-slave-simple" )
#define POLL_FOREVER        ( -1  )

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

    MAX_USER_CMD

} USER_CMD;

/*****************************************************************************/
/* Local variables                                                           */
/*****************************************************************************/

static struct i2c_data  xTxData = { 0 };
static struct i2c_event xRxData = { 0 };
static bool bQuit               = false;
static bool bStopPolling        = false;

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
 * @brief Poll the I2C slave for any events.
 * 
 * @param iDevice File descriptor of the I2C slave device.
 * 
 * @returns None
 */
static void vPollSlaveLoop( int iDevice );

/**
 * @brief Set the slave I2C response data.
 * 
 * @param iDevice File descriptor of the I2C slave device
 * 
 * @returns None
 */
static void vSetSlaveData( int iDevice );

/**
 * @brief Get the slave I2C data written by a master. This does not check
 *   if a transaction was completed fully or if there even is any data. Also,
 *   the last_event on the driver side does not get cleared.
 * 
 * @param iDevice File descriptor of the I2C slave device
 * 
 * @returns None
 */
static void vGetSlaveData( int iDevice );

/**
 * @brief Get the last event/events received by the I2C slave.
 * 
 * @param iDevice File descriptor of the I2C slave device
 * 
 * @returns None
 * 
 */
static void vGetSlaveEvent( int iDevice );

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
    NEW_LINE;
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
static void vPollSlaveLoop( int iDevice )
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
                vGetSlaveEvent( iDevice );
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
static void vSetSlaveData( int iDevice )
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
static void vGetSlaveData( int iDevice )
{
    if( INVALID_FD != iDevice )
    {
        int iRet = 0;
        SMBUS_PRINTF( "Reading value from driver\r\n" );
        iRet = ioctl( iDevice, I2C_SLAVE_GET_DATA, ( uint8_t* )&xRxData.wr_data );
        SMBUS_PRINTF( "IOCTL complete - got %d byte(s) with return value %d\r\n", xRxData.wr_data.data_size, iRet );
        vDisplayBuffer( &xRxData.wr_data.data[ 0 ], xRxData.wr_data.data_size );
    }
    else
    {
        SMBUS_PRINTF_ERR( "Invalid device file\r\n" );
    }
}

/**
 * @brief Get the last slave event.
 */
static void vGetSlaveEvent( int iDevice )
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

        vDisplayBuffer( &xRxData.wr_data.data[ 0 ], xRxData.wr_data.data_size );
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
    int iDevice = INVALID_FD;
    uint8_t ucInput = USER_CMD_QUIT;

    /* Try to open the device file. */
    SMBUS_PRINTF( "Opening device...\r\n" );
    iDevice = open( DEVICE_FILE, O_RDWR | O_NONBLOCK );

    if( INVALID_FD != iDevice )
    {
        vDisplayCommands();

        do
        {
            ucInput = ucInputByte( "Enter command", false, false );

            switch( ucInput )
            {
                case USER_CMD_SET_DATA:
                {
                    vSetSlaveData( iDevice );
                    break;
                }

                case USER_CMD_GET_DATA:
                {
                    vGetSlaveData( iDevice );
                    break;
                }

                case USER_CMD_POLL:
                {
                    vPollSlaveLoop( iDevice );
                    break;
                }

                case USER_CMD_HELP:
                {
                    vDisplayCommands();
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

        SMBUS_PRINTF( "Closing device file...\r\n" );
        close( iDevice );
    }
    else
    {
        SMBUS_PRINTF_ERR( "Cannot open device file...\r\n" );
    }
}
