#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "smbus.h"
#include "test_utils.h"
#include "smbus_protocols.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/

#define INVALID_FD            ( -1 )
#define LINUX_ERROR           ( -1 )
#define LINUX_OK              ( 0  )

#define SMBUS_FILENAME_LEN    ( 30 )

#define READ                  ( 1U << 0 )
#define WRITE                 ( 1U << 1 )

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/**
 * @enum USER_CMD
 * @brief List of test commands
 */
typedef enum USER_CMD
{
    USER_CMD_QUIT         = 0,

    USER_CMD_SEND_BYTE    = SMBUS_PROTOCOL_SEND_BYTE,
    USER_CMD_RECEIVE_BYTE = SMBUS_PROTOCOL_RECEIVE_BYTE,
    USER_CMD_WRITE_BYTE   = SMBUS_PROTOCOL_WRITE_BYTE,
    USER_CMD_READ_BYTE    = SMBUS_PROTOCOL_READ_BYTE,

    USER_CMD_HELP         = ( uint8_t )( -1 )

} USER_CMD;

/*****************************************************************************/
/* Local variables                                                           */
/*****************************************************************************/

static bool bQuit                          = false;
static int iDevice                         = INVALID_FD;
static uint16_t usSlaveAddr                = 0;
static uint8_t pucWrData[ SMBUS_MAX_DATA ] = { 0 };

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
 * @brief Perform a single SMBus transaction with preloaded data.
 * 
 * @param eProtocol The SMBus protocol type of this transaction
 *
 * @returns None
 */
static void vDoSMBusTransaction( SMBUS_PROTOCOL_COMMANDS eProtocol );

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
    PRINT_ENUM( USER_CMD_SEND_BYTE );
    PRINT_ENUM( USER_CMD_RECEIVE_BYTE );
    PRINT_ENUM( USER_CMD_WRITE_BYTE);
    PRINT_ENUM( USER_CMD_READ_BYTE );
    NEW_LINE;
}

/**
 * @brief Perform an SMBus transaction
 */
static void vDoSMBusTransaction( SMBUS_PROTOCOL_COMMANDS eProtocol )
{
    if( INVALID_FD != iDevice )
    {
        uint16_t usWrLen = 0;
        uint16_t usRdLen = 0;
        uint8_t pucWrBuffer[ SMBUS_MAX_BUFFER ] = { 0 };
        uint8_t pucRdBuffer[ SMBUS_MAX_BUFFER ] = { 0 };

        uint8_t ucNumMessages = 0;
        struct i2c_msg xMessages[ 2 ] = { 0 };
        struct i2c_rdwr_ioctl_data xData = { 0 };

        uint8_t iTransactionType = 0;

        switch( eProtocol )
        {
            case SMBUS_PROTOCOL_WRITE_BYTE:
            {
                iTransactionType = WRITE;
                pucWrBuffer[ usWrLen++ ] = ( uint8_t )SMBUS_PROTOCOL_WRITE_BYTE;  /* command */
                pucWrBuffer[ usWrLen++ ] = pucWrData[ 0 ];  /* data byte */
                break;
            }

            case SMBUS_PROTOCOL_READ_BYTE:
            {
                iTransactionType = WRITE | READ;
                pucWrBuffer[ usWrLen++ ] = ( uint8_t )SMBUS_PROTOCOL_READ_BYTE;  /* command */
                usRdLen = 1;  /* Reading back 1 byte. */
                break;
            }

            case SMBUS_PROTOCOL_SEND_BYTE:
            {
                iTransactionType = WRITE;
                pucWrBuffer[ usWrLen++ ] = ( uint8_t )SMBUS_PROTOCOL_SEND_BYTE;  /* data byte */
                break;
            }

            case SMBUS_PROTOCOL_RECEIVE_BYTE:
            {
                iTransactionType = READ;
                usRdLen = 1;  /* Reading 1 byte with no command. */
                break;
            }

            default:
            {
                break;
            }
        }

        if( iTransactionType & WRITE )
        {
            xMessages[ ucNumMessages ].addr = usSlaveAddr;
            xMessages[ ucNumMessages ].flags = 0;
            xMessages[ ucNumMessages ].len = usWrLen;
            xMessages[ ucNumMessages ].buf = &pucWrBuffer[ 0 ];
            ucNumMessages++;
        }

        if( iTransactionType & READ )
        {
            xMessages[ ucNumMessages ].addr = usSlaveAddr;
            xMessages[ ucNumMessages ].flags = I2C_M_RD;
            xMessages[ ucNumMessages ].len = usRdLen;
            xMessages[ ucNumMessages ].buf = &pucRdBuffer[ 0 ];
            ucNumMessages++;
        }

        xData.msgs = xMessages;
        xData.nmsgs = ucNumMessages;

        SMBUS_PRINTF(
            "Doing I2C transaction with parameters:\r\n"
            "    Command = 0x%02hhX\r\n"
            "    Wr Len  = %d\r\n"
            "    Rd Len  = %d\r\n"
            "    Slave   = 0x%02hhX\r\n",
            ( uint8_t )eProtocol, usWrLen, usRdLen, usSlaveAddr
        );

        if( LINUX_ERROR != ioctl( iDevice, I2C_RDWR, &xData ) )
        {
            SMBUS_PRINTF( "OK. Data sent!\r\n" );
            vDisplayBuffer( pucWrBuffer, usWrLen, "pucWrBuffer" );
            vDisplayBuffer( pucRdBuffer, usRdLen, "pucRdBuffer" );
        }
        else
        {
            SMBUS_PRINTF_ERR( "ERROR. Could not send data!\r\n" );
        }
    }
    else
    {
        SMBUS_PRINTF_ERR( "Error. Invalid I2C device.\r\n" );
    }
}

/**
 * @brief Run the main master loop.
 */
int main()
{
    uint8_t ucCmd   = 0;
    uint8_t ucInput = USER_CMD_QUIT;

    SMBUS_PRINTF( "Preloading Wr data\r\n" );

    /* Set write data. */
    int i = 0;
    for( i = 0; i < SMBUS_MAX_DATA; i++ )
    {
        pucWrData[ i ] = ( uint8_t )i;
    }

    /* Open the i2c device. */
    SMBUS_PRINTF( "Opening I2C bus\r\n" );
    ucInput = ucInputByte( "Enter bus number", false , false );

    char pcSMBusDevName[ SMBUS_FILENAME_LEN ] = { 0 };
    snprintf( pcSMBusDevName, SMBUS_FILENAME_LEN - 1, "/dev/i2c-%d", ucInput );
    iDevice = open( pcSMBusDevName, O_RDWR );

    if( INVALID_FD != iDevice )
    {
        SMBUS_PRINTF( "I2C bus fd = %d\r\n", iDevice );

        /* Get the slave address. */
        usSlaveAddr = ucInputByte( "Enter slave address", true , false );
        SMBUS_PRINTF( "Slave address set to 0x%02hhX\r\n", usSlaveAddr );

        /* Now we can execute user commands. */
        vDisplayCommands();

        do
        {
            ucInput = ucInputByte( "Enter command", false, false );

            switch( ucInput )
            {
                case USER_CMD_RECEIVE_BYTE:
                case USER_CMD_READ_BYTE:
                case USER_CMD_SEND_BYTE:
                case USER_CMD_WRITE_BYTE:
                {
                    vDoSMBusTransaction( ( SMBUS_PROTOCOL_COMMANDS )ucInput );
                    break;
                }

                case USER_CMD_QUIT:
                {
                    bQuit = true;
                    break;
                }

                case USER_CMD_HELP:
                {
                    vDisplayCommands();
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

        /* Close the i2c device. */
        SMBUS_PRINTF( "Closing I2C bus\r\n" );
        close( iDevice );
    }
    else
    {
        SMBUS_PRINTF_ERR( "Could not open I2C bus\r\n" );
    }
}
