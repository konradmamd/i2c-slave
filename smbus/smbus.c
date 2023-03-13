/**
 * @file smbus.c
 * @author Konrad Mosoczy (konradm@amd.com)
 * @brief OpenBMC SMBus API implementation
 * @date 2023-03-08
 * 
 * Copyright (c) 2023-present Advanced Micro Devices, Inc. All rights reserved.
 * 
 */

 /* 
  * NOTE: See https://www.kernel.org/doc/Documentation/i2c/dev-interface for reference.
  * To summarise, there are mulitiple approaches to performing SMBus commands:
  *     - Using read/write calls for raw I2C (does not support repeated start)
  *     - Using I2C_SMBUS ioctl or the related i2c_smbus_XXX functions
  *     - Using I2C_RDWR ioctl calls for raw I2C with repeated start support
  */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

/* Standard includes */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <endian.h>
#include <sys/ioctl.h>

/* External includes */
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>  /* Master interface */

/* i2c-slave includes */
#include "smbus.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/

#define UNUSED( x )               ( void )( x )
#define OPERATION_NOT_SUPPORTED   return SMBUS_STATUS_ERROR
#define UNINIT_ADDR               ( 0x00 )
#define SMBUS_FILENAME_LEN        ( 30   )

/*
 * See http://www.merkles.com/Using_I2C/5_struct_i2c_msg.html 
 * See also linux/i2c.h and linux/i2c-dev.h
 */
#define I2C_MSG_NO_FLAGS          ( 0 )

/*****************************************************************************/
/* Enums, Structs                                                            */
/*****************************************************************************/

/**
 * @enum  LINUX_STATUS
 * @brief Enum wrapper around Linux I2C API return codes
 */
typedef enum LINUX_STATUS
{
    /* For open, close. */
    LINUX_STATUS_VALID_HANDLE    = 0,
    LINUX_STATUS_INVALID_HANDLE  = -1,

    /* For ioctl. */
    LINUX_STATUS_RW_OK           = 0,
    LINUX_STATUS_RW_ERROR        = -1

} LINUX_STATUS;

/**
 * @enum  TRANSACTION_TYPE
 * @brief Bitflags to set the type of SMBus transaction being performed
 * 
 */
typedef enum TRANSACTION_TYPE
{
    TRANSACTION_TYPE_READ  = 0x01,
    TRANSACTION_TYPE_WRITE = 0x02
    
} TRANSACTION_TYPE;

/**
 * @enum  DATA_SIZE
 * @brief SMBus data size constants
 */
typedef enum DATA_SIZE
{
    DATA_SIZE_EMPTY  = 0,
    DATA_SIZE_BYTE   = 1,
    DATA_SIZE_WORD   = 2,
    DATA_SIZE_32_BIT = 4,
    DATA_SIZE_64_BIT = 8

} DATA_SIZE;

/**
 * @struct SMBUS_DATA
 * @brief  Data related to a single SMBus transaction
 *
 * @note   We need two separate buffers - one for Wr data and one for Rd data
 *         because we must send all messages in a single transaction/ioctl call.
 *         This is because a stop bit will be automatically sent after the last
 *         message, meaning we can't split messages into multiple ioctl calls
 *         for a single SMBus transaction. We will only ever need two messages
 *         at most.
 */
typedef struct SMBUS_DATA
{
    uint16_t   usAddr;
    bool       bPec;

    /* Data following a WR bit. Data sent if master, data received if slave. */
    uint16_t   usWrLen;
    uint8_t    pucWrBuffer[ SMBUS_MAX_BUFFER ];

    /* Data following a RD bit. Data received if master, data sent if slave. */
    uint16_t   usRdLen;
    uint8_t    pucRdBuffer[ SMBUS_MAX_BUFFER ];

} SMBUS_DATA;

/*****************************************************************************/
/* Function declarations                                                     */
/*****************************************************************************/

/**
 * @brief Perform a single I2C transaction.
 * 
 * @param eTransactionType     Type of transaction to perform
 *
 * @return SMBUS_STATUS_OK     Transaction was successful
 *         SMBUS_STATUS_ERROR  Transaction was unsuccessful
 */
static int iDoTransaction( TRANSACTION_TYPE eTransactionType );

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

static SMBUS_DATA xConSMBusData = {
    .usAddr        = 0,
    .bPec          = false,
    .usWrLen       = 0,
    .usRdLen       = 0,
    .pucRdBuffer   = { 0 },
    .pucWrBuffer   = { 0 }
};

static int iBusHandle = LINUX_STATUS_INVALID_HANDLE;

/*****************************************************************************/
/* Public Function Definitions - General                                     */
/*****************************************************************************/

/**
 * @brief Initialise SMBus device
 */
int smbus_init( int iFile, int iBitRateKHz )
{
    UNUSED( iBitRateKHz );

    int iStatus = SMBUS_STATUS_ERROR;

    char pcSMBusDevName[ SMBUS_FILENAME_LEN ] = { 0 };
    snprintf( pcSMBusDevName, SMBUS_FILENAME_LEN - 1, "/dev/i2c-%d", iFile );

    if( LINUX_STATUS_INVALID_HANDLE == iBusHandle )
    {
        int iTempHandle = open( pcSMBusDevName, O_RDWR );

        if( LINUX_STATUS_INVALID_HANDLE != iTempHandle )
        {
            iBusHandle = iTempHandle;
            iStatus = SMBUS_STATUS_OK;
        }
    }

    return iStatus;
}

/**
 * @brief De-Initialise SMBus device
 */
int smbus_deinit( void )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( LINUX_STATUS_INVALID_HANDLE != close( iBusHandle ) )
    {
        iStatus = SMBUS_STATUS_OK;
        iBusHandle = LINUX_STATUS_INVALID_HANDLE;
    }

    return iStatus;
}

/**
 * @brief This function updates bInvalidPec variable in static SMBUS_TEST_CTRL struct
 */
void smbus_inject_invalid_pec( bool bInvalidPec )
{
}

/**
 * @brief This function updates xInvalidBlockSize variable in static SMBUS_TEST_CTRL struct
 */
void smbus_inject_invalid_block_size( SMBUS_INJECT_BLOCK_SIZE_ERROR xInvalidBlockSize )
{
}

/**
 * @brief Call SMBus command "Send byte"
 */
int smbus_send_byte( uint8_t ucAddr, uint8_t ucData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( LINUX_STATUS_INVALID_HANDLE != iBusHandle )
    {
        uint16_t usWrIndex = 0;

        xConSMBusData.usAddr                     = ( uint8_t )ucAddr;
        xConSMBusData.bPec                       = bPec;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ucData;
        xConSMBusData.usWrLen                    = usWrIndex;
        xConSMBusData.usRdLen                    = DATA_SIZE_EMPTY;

        iStatus = iDoTransaction( TRANSACTION_TYPE_WRITE );
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Receive byte"
 */
int smbus_receive_byte( uint8_t ucAddr, uint8_t* pucData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( LINUX_STATUS_INVALID_HANDLE != iBusHandle )
    {
        xConSMBusData.usAddr         = ( uint8_t )ucAddr;
        xConSMBusData.bPec           = bPec;
        xConSMBusData.usWrLen        = DATA_SIZE_EMPTY;
        xConSMBusData.usRdLen        = DATA_SIZE_BYTE;

        memset( xConSMBusData.pucRdBuffer, 0xFF, SMBUS_MAX_BUFFER );

        if( SMBUS_STATUS_OK == iDoTransaction( TRANSACTION_TYPE_READ ) )
        {
            *pucData = ( uint8_t )( xConSMBusData.pucRdBuffer[ 0 ] );
            iStatus = SMBUS_STATUS_OK;
        }
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Write byte"
 */
int smbus_write_byte( uint8_t ucAddr, uint8_t ucCmd, uint8_t ucData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( LINUX_STATUS_INVALID_HANDLE != iBusHandle )
    {
        uint16_t usWrIndex = 0;

        xConSMBusData.usAddr                     = ( uint8_t )ucAddr;
        xConSMBusData.bPec                       = bPec;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ucCmd;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ucData;
        xConSMBusData.usWrLen                    = usWrIndex;
        xConSMBusData.usRdLen                    = DATA_SIZE_EMPTY;

        iStatus = iDoTransaction( TRANSACTION_TYPE_WRITE );
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Read byte"
 */
int smbus_read_byte( uint8_t ucAddr, uint8_t ucCmd, uint8_t* pucData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( LINUX_STATUS_INVALID_HANDLE != iBusHandle )
    {
        uint16_t usWrIndex = 0;

        xConSMBusData.usAddr                     = ( uint8_t )ucAddr;
        xConSMBusData.bPec                       = bPec;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ucCmd;
        xConSMBusData.usWrLen                    = usWrIndex;
        xConSMBusData.usRdLen                    = DATA_SIZE_BYTE;

        memset( xConSMBusData.pucRdBuffer, 0xFF, SMBUS_MAX_BUFFER );

        if( SMBUS_STATUS_OK == iDoTransaction( TRANSACTION_TYPE_READ | TRANSACTION_TYPE_WRITE ) )
        {
            *pucData = ( uint8_t )( xConSMBusData.pucRdBuffer[ 0 ] );
        }
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Write word"
 */
int smbus_write_word( uint8_t ucAddr, uint8_t ucCmd, uint16_t usData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( LINUX_STATUS_INVALID_HANDLE != iBusHandle )
    {
        uint16_t usWrIndex = 0;
        uint16_t usLeData = htole16( usData );

        xConSMBusData.usAddr                     = ( uint8_t )ucAddr;
        xConSMBusData.bPec                       = bPec;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ucCmd;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( usLeData );
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( usLeData >> 8);
        xConSMBusData.usWrLen                    = usWrIndex;
        xConSMBusData.usRdLen                    = DATA_SIZE_EMPTY;

        iStatus = iDoTransaction( TRANSACTION_TYPE_WRITE );
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Read Word"
 */
int smbus_read_word( uint8_t ucAddr, uint8_t ucCmd, uint16_t* pusData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) &&
        ( NULL != pusData ) )
    {
        uint16_t usWrIndex = 0;

        xConSMBusData.usAddr                     = ( uint8_t )ucAddr;
        xConSMBusData.bPec                       = bPec;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ucCmd;
        xConSMBusData.usWrLen                    = usWrIndex;
        xConSMBusData.usRdLen                    = DATA_SIZE_WORD;

        memset( xConSMBusData.pucRdBuffer, 0xFF, SMBUS_MAX_BUFFER );

        if( SMBUS_STATUS_OK == iDoTransaction( TRANSACTION_TYPE_READ | TRANSACTION_TYPE_WRITE ) )
        {
            uint16_t usLeData = ( ( uint16_t )( xConSMBusData.pucRdBuffer[ 1 ] << 8 ) );
            usLeData         |=   ( uint16_t )( xConSMBusData.pucRdBuffer[ 0 ] );

            *pusData = le16toh( usLeData );
            iStatus = SMBUS_STATUS_OK;
        }
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Write 32"
 */
int smbus_write_32( uint8_t ucAddr, uint8_t ucCmd, uint32_t ulData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( LINUX_STATUS_INVALID_HANDLE != iBusHandle )
    {
        uint16_t usWrIndex = 0;
        uint32_t ulLeData = htole32( ulData );

        xConSMBusData.usAddr                     = ( uint8_t )ucAddr;
        xConSMBusData.bPec                       = bPec;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ucCmd;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ulLeData );
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ulLeData >> 8);
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ulLeData >> 16);
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ulLeData >> 24);
        xConSMBusData.usWrLen                    = usWrIndex;
        xConSMBusData.usRdLen                    = DATA_SIZE_EMPTY;

        iStatus = iDoTransaction( TRANSACTION_TYPE_WRITE );
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Read 32"
 */
int smbus_read_32( uint8_t ucAddr, uint8_t ucCmd, uint32_t* pulData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) &&
        ( NULL != pulData ) )
    {
        uint16_t usWrIndex = 0;

        xConSMBusData.usAddr                     = ( uint8_t )ucAddr;
        xConSMBusData.bPec                       = bPec;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ucCmd;
        xConSMBusData.usWrLen                    = usWrIndex;
        xConSMBusData.usRdLen                    = DATA_SIZE_32_BIT;

        memset( xConSMBusData.pucRdBuffer, 0xFF, SMBUS_MAX_BUFFER );

        if( SMBUS_STATUS_OK == iDoTransaction( TRANSACTION_TYPE_READ | TRANSACTION_TYPE_WRITE ) )
        {
            uint32_t ulLeData = ( ( uint32_t )( xConSMBusData.pucRdBuffer[ 3 ] ) << 24 );
            ulLeData         |= ( ( uint32_t )( xConSMBusData.pucRdBuffer[ 2 ] ) << 16 );
            ulLeData         |= ( ( uint32_t )( xConSMBusData.pucRdBuffer[ 1 ] ) << 8 );
            ulLeData         |=   ( uint32_t )( xConSMBusData.pucRdBuffer[ 0 ] );

            *pulData = le32toh( ulLeData );
            iStatus = SMBUS_STATUS_OK;
        }
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Write 64"
 */
int smbus_write_64( uint8_t ucAddr, uint8_t ucCmd, uint64_t ullData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( LINUX_STATUS_INVALID_HANDLE != iBusHandle )
    {
        uint16_t usWrIndex = 0;
        uint64_t ullLeData = htole64( ullData );

        xConSMBusData.usAddr                     = ( uint8_t )ucAddr;
        xConSMBusData.bPec                       = bPec;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ucCmd;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ullLeData );
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ullLeData >> 8 );
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ullLeData >> 16 );
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ullLeData >> 24 );
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ullLeData >> 32 );
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ullLeData >> 40 );
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ullLeData >> 48 );
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ( uint8_t )( ullLeData >> 56 );
        xConSMBusData.usWrLen                    = usWrIndex;
        xConSMBusData.usRdLen                    = DATA_SIZE_EMPTY;

        iStatus = iDoTransaction( TRANSACTION_TYPE_WRITE );
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Read 64"
 */
int smbus_read_64( uint8_t ucAddr, uint8_t ucCmd, uint64_t* pullData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) &&
        ( NULL != pullData ) )
    {
        uint16_t usWrIndex = 0;

        xConSMBusData.usAddr                     = ( uint8_t )ucAddr;
        xConSMBusData.bPec                       = bPec;
        xConSMBusData.pucWrBuffer[ usWrIndex++ ] = ucCmd;
        xConSMBusData.usWrLen                    = usWrIndex;
        xConSMBusData.usRdLen                    = DATA_SIZE_64_BIT;

        memset( xConSMBusData.pucRdBuffer, 0xFF, SMBUS_MAX_BUFFER );

        if( SMBUS_STATUS_OK == iDoTransaction( TRANSACTION_TYPE_READ | TRANSACTION_TYPE_WRITE ) )
        {
            uint64_t ullLeData = ( ( uint64_t )( xConSMBusData.pucRdBuffer[ 7 ] ) << 56 );
            ullLeData         |= ( ( uint64_t )( xConSMBusData.pucRdBuffer[ 6 ] ) << 48 );
            ullLeData         |= ( ( uint64_t )( xConSMBusData.pucRdBuffer[ 5 ] ) << 40 );
            ullLeData         |= ( ( uint64_t )( xConSMBusData.pucRdBuffer[ 4 ] ) << 32 );
            ullLeData         |= ( ( uint64_t )( xConSMBusData.pucRdBuffer[ 3 ] ) << 24 );
            ullLeData         |= ( ( uint64_t )( xConSMBusData.pucRdBuffer[ 2 ] ) << 16 );
            ullLeData         |= ( ( uint64_t )( xConSMBusData.pucRdBuffer[ 1 ] ) << 8 );
            ullLeData         |=   ( uint64_t )( xConSMBusData.pucRdBuffer[ 0 ] );

            *pullData = le64toh( ullLeData );
            iStatus = SMBUS_STATUS_OK;
        }
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Block write"
 */
int smbus_block_write( uint8_t ucAddr, uint8_t ucCmd, uint8_t* pucData, uint8_t ucLength, bool bPec )
{
    // int iStatus = SMBUS_STATUS_ERROR;

    // if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) &&
    //     ( NULL != pucData ) && ( SMBUS_MAX_DATA >= ucLength ) &&
    //     ( SMBUS_STATUS_OK == iClaimSlave( ucAddr, bPec ) ) )
    // {
    //     if( LINUX_STATUS_RW_OK == i2c_smbus_write_block_data( iBusHandle, ucCmd, ucLength, pucData ) )
    //     {
    //         iStatus = SMBUS_STATUS_OK;
    //     }
    // }

    // return iStatus;
}

/**
 * @brief Call SMBus command "Block read"
 */
int smbus_block_read( uint8_t ucAddr, uint8_t ucCmd, uint8_t* pucData, uint8_t* pucLength, bool bPec )
{
    // int iStatus = SMBUS_STATUS_ERROR;

    // if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) && 
    //     ( NULL != pucData ) && ( NULL != pucLength ) &&
    //     ( SMBUS_STATUS_OK == iClaimSlave( ucAddr, bPec ) ) )
    // {
    //     int iBytesRead = i2c_smbus_read_block_data( iBusHandle, ucCmd, pucData );

    //     if( LINUX_STATUS_RW_ERROR != iBytesRead )
    //     {
    //         *pucLength = ( uint8_t )iBytesRead;
    //         iStatus = SMBUS_STATUS_OK;
    //     }
    //     else
    //     {
    //         *pucLength = 0;
    //     }
    // }

    // return iStatus;
}

/**
 * @brief Call SMBus command "Process call"
 */
int smbus_process_call( uint8_t ucAddr, uint8_t ucCmd, uint16_t usWrData, uint16_t* pusRdData, bool bPec )
{
    // int iStatus = SMBUS_STATUS_ERROR;

    // if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) && 
    //     ( NULL != pusRdData ) &&
    //     ( SMBUS_STATUS_OK == iClaimSlave( ucAddr, bPec ) ) )
    // {
    //     int iTempWord = i2c_smbus_process_call( iBusHandle, ucCmd, usWrData );

    //     if( LINUX_STATUS_RW_ERROR != iTempWord )
    //     {
    //         *pusRdData = ( uint16_t )iTempWord;
    //         iStatus = SMBUS_STATUS_OK;
    //     }
    // }

    // return iStatus;
}

/**
 * @brief Call SMBus command "Block Write/Read Process call"
 */
int smbus_block_process_call( uint8_t ucAddr, uint8_t ucCmd,
                              uint8_t* pucWrData, uint8_t ucWrLen,
                              uint8_t* pucRdData, uint8_t* pucRdLen,
                              bool bPec )
{
    // int iStatus = SMBUS_STATUS_ERROR;

    // if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) && 
    //     ( NULL != pucWrData ) && ( NULL != pucRdData ) &&
    //     ( NULL != pucRdLen ) && ( SMBUS_MAX_DATA >= ucWrLen ) &&
    //     ( SMBUS_STATUS_OK == iClaimSlave( ucAddr, bPec ) ) )
    // {
    //     /* Create a temp buffer so we don't override `pucWrData`. */
    //     uint8_t pucTempBuffer[ SMBUS_MAX_BUFFER ] = { 0 };
    //     memcpy( pucTempBuffer, pucWrData, ucWrLen );

    //     int iBytesRead = i2c_smbus_block_process_call( iBusHandle, ucCmd, ucWrLen, pucTempBuffer );

    //     if( LINUX_STATUS_RW_ERROR != iBytesRead )
    //     {
    //         *pucRdLen = ( uint8_t )iBytesRead;
    //         memcpy( pucRdData, pucTempBuffer, *pucRdLen );
    //         iStatus = SMBUS_STATUS_OK;
    //     }
    // }

    // return iStatus;
}

/*****************************************************************************/
/* Public function implementations - Target                                  */
/*****************************************************************************/

/**
 * @brief Enable an SMBus target at a fixed address
 */
int smbus_target_enable( uint8_t ucAddr )
{
    OPERATION_NOT_SUPPORTED;
}

/**
 * @brief Disable the SMBus target
 */
int smbus_target_disable( void )
{
    OPERATION_NOT_SUPPORTED;
}

/**
 * @brief Prepare data for target transmission
 */
int smbus_target_set_data( uint8_t* pucData,
                           uint8_t ucLength,
                           SMBUS_TARGET_PEC_INFO* pxPecInfo )
{
    OPERATION_NOT_SUPPORTED;
}

/**
 * @brief Retrieve data sent to target
 */
int smbus_target_get_data( uint8_t* pucData,
                           uint16_t* pusLength,
                           SMBUS_TARGET_GET_DATA_FILTER eDataFilter,
                           int iTimeoutMs,
                           bool bPecExpected )
{
    OPERATION_NOT_SUPPORTED;
}

/*****************************************************************************/
/* Local function implementations                                            */
/*****************************************************************************/

static int iDoTransaction( TRANSACTION_TYPE eTransactionType )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( LINUX_STATUS_INVALID_HANDLE != iBusHandle )
    {
        uint8_t ucNumMessages = 0;
        struct i2c_msg xMessages[ 2 ] = { 0 };
        struct i2c_rdwr_ioctl_data xData = { 0 };

        if( eTransactionType & TRANSACTION_TYPE_WRITE )
        {
            /* Write message is always first. */
            xMessages[ ucNumMessages ].addr = xConSMBusData.usAddr;
            xMessages[ ucNumMessages ].flags = I2C_MSG_NO_FLAGS;
            xMessages[ ucNumMessages ].len = xConSMBusData.usWrLen;
            xMessages[ ucNumMessages ].buf = &xConSMBusData.pucWrBuffer[ 0 ];
            ucNumMessages++;
        }

        if( eTransactionType & TRANSACTION_TYPE_READ )
        {
            xMessages[ ucNumMessages ].addr = xConSMBusData.usAddr;
            xMessages[ ucNumMessages ].flags = I2C_M_RD;
            xMessages[ ucNumMessages ].len = xConSMBusData.usRdLen;
            xMessages[ ucNumMessages ].buf = &xConSMBusData.pucRdBuffer[ 0 ];
            ucNumMessages++;
        }

        xData.msgs = xMessages;
        xData.nmsgs = ucNumMessages;

        iStatus = ( LINUX_STATUS_RW_ERROR == ioctl( iBusHandle, I2C_RDWR, &xData ) ) ?
                  ( SMBUS_STATUS_ERROR ) : ( SMBUS_STATUS_OK );
    }

    return iStatus;
}
