/**
 * @file test_slave.c
 * @author Konrad Mosoczy (konradm@amd.com)
 * @brief App to test I2C slave functionality on the OpenBMC
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
#include <time.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>

#include "i2c-slave-eeprom.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/

#define UNUSED( x )         ( void )( x )

#define INVALID_FD          ( -1  )
#define DEVICE_FILE         ( "/dev/i2c-slave-simple" )
#define POLL_FOREVER        ( -1  )

#define BUFFER_PRINT_WIDTH  ( 8 )
#define SMBUS_PRINTF        { struct timespec T = { 0 }; clock_gettime( CLOCK_REALTIME, &T ); \
                            struct tm *t = localtime( &T.tv_sec );                            \
                            printf( "[ %02d:%02d:%02d:%03ld ] ",                              \
                                ( t->tm_hour ),                                               \
                                ( t->tm_min ),                                                \
                                ( t->tm_sec ),                                                \
                                ( T.tv_nsec/1000000 ) ); }                                    \
                            printf

#define PRINT_ENUM(x)      SMBUS_PRINTF( "%-40s: %d\r\n", #x, ( char )x )
#define SMBUS_PRINTF_DBG   SMBUS_PRINTF( "%d - ", __LINE__ ); printf
#define SMBUS_PRINTF_ERR   SMBUS_PRINTF_DBG( "ERROR - " ); printf
#define NEW_LINE           printf( "\r\n" );

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
 * @brief Displays data buffer
 *
 * @param pucBuffer Buffer to display
 * @param usLength Number of bytes in buffer
 *
 * @return N/A
 */
static void vDisplayBuffer( uint8_t *pucBuffer, uint16_t usLength );

/**
 * @brief Read a byte from stdin
 *
 * @param sPrompt Prompt to display to user
 * @param bHex TRUE if expected input value is b16, otherwise it will be b10
 * @param bNewLine TRUE if a newline should ne printed after the prompt
 *
 * @return Single byte, read from stdin (0 is the default value if there are errors)
 */
static uint8_t ucInputByte( const char *sPrompt, bool bHex, bool bNewLine );

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
 * @brief Read a byte from stdin
 */
static uint8_t ucInputByte( const char *sPrompt, bool bHex, bool bNewLine )
{
    int iRet = 0;
    uint8_t ucByte = 0;

    if( NULL != sPrompt )
    {
        do
        {
            fflush( stdin );
            printf( "%s (%s): ", sPrompt, ( true == bHex )?( "hex" ):( "dec" ) );
            if( true == bNewLine )
            {
                printf( "\r\n" );
            }
            if( true == bHex )
            {
                iRet = scanf( " %hhx", &ucByte );
                UNUSED( iRet );
            }
            else
            {
                iRet = scanf( " %hhd", &ucByte );
                UNUSED( iRet );
            }

            fflush( stdin );
        
        } while( 0xFF < ucByte );
    }

    return ucByte;
}

/**
 * @brief Displays data buffer
 */
static void vDisplayBuffer( uint8_t *pucBuffer, uint16_t usLength )
{
    if( NULL != pucBuffer )
    {
        int i = 0;

        SMBUS_PRINTF( "Data:\r\n" );
        for( i = 0; i < usLength; i++ )
        {
            if( 0 == ( i % BUFFER_PRINT_WIDTH ) )
            {
                if( 0 != i )
                {
                    printf( " [ %02X ]\r\n", i - 1 );
                }
                SMBUS_PRINTF( "\t[ %02X ] ", i );
            }
            printf( " %02X ", pucBuffer[ i ] );
        }
        for( i = i; ( 0 != ( i % BUFFER_PRINT_WIDTH ) ); i++ )
        {
            printf( " -- " );
        }
        printf( " [ %02X ]\r\n", i - 1 );
    }
    else
    {
        SMBUS_PRINTF_ERR( "Invalid data\r\n" );
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
