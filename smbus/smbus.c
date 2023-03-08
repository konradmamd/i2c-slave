/**
 * @file smbus.c
 * @author Konrad Mosoczy (konradm@amd.com)
 * @brief OpenBMC SMBus API implementation
 * @date 2023-03-08
 * 
 * Copyright (c) 2023-present Advanced Micro Devices, Inc. All rights reserved.
 * 
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

/* Standard includes */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
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

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/**
 * @enum LINUX_STATUS
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

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

static uint8_t ucCurrentSlaveAddr = UNINIT_ADDR;
static int iBusHandle = LINUX_STATUS_INVALID_HANDLE;

/*****************************************************************************/
/* Local Function Declarations                                               */
/*****************************************************************************/

/**
 * @brief Try to claim a slave device at a specific address,
 *   only if it is not claimed already.
 * 
 * @param ucAddr Address of the slave device
 * @param bPec   Enable/disable PEC for this device
 * 
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
static int iClaimSlave( uint8_t ucAddr, bool bPec );

/*****************************************************************************/
/* Local Function Definitions                                                */
/*****************************************************************************/

/**
 * @brief Claim an i2c slave device with ioctl calls.
 */
static int iClaimSlave( uint8_t ucAddr, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( LINUX_STATUS_INVALID_HANDLE != iBusHandle )
    {
        /* Make I2C_SLAVE ioctl call. */
        if( ucCurrentSlaveAddr != ucAddr )
        {
            if( LINUX_STATUS_RW_ERROR != ioctl( iBusHandle, I2C_SLAVE, ucAddr ) )
            {
                ucCurrentSlaveAddr = ucAddr;
                iStatus = SMBUS_STATUS_OK;
            }
        }
        else
        {
            iStatus = SMBUS_STATUS_OK;
        }

        /* Enable/disable PEC. TODO: Test this. */
        if( SMBUS_STATUS_OK == iStatus )
        {
            iStatus = SMBUS_STATUS_ERROR;
            int iPecSelect = ( bPec ) ? ( ucAddr ) : ( 0 );  /* 0 to disable, `ucAddr` otherwise */

            if( LINUX_STATUS_RW_ERROR != ioctl( iBusHandle, I2C_PEC, iPecSelect ) )
            {
                iStatus = SMBUS_STATUS_OK;
            }
        }
    }

    return iStatus;
}

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
        ucCurrentSlaveAddr = UNINIT_ADDR;
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Send byte"
 */
int smbus_send_byte( uint8_t ucAddr, uint8_t ucData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) &&
        ( SMBUS_STATUS_OK == iClaimSlave( ucAddr, bPec ) ) )
    {
        if( LINUX_STATUS_RW_OK == i2c_smbus_write_byte( iBusHandle, ucData ) )
        {
            iStatus = SMBUS_STATUS_OK;
        }
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Receive byte"
 */
int smbus_receive_byte( uint8_t ucAddr, uint8_t* pucData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) && 
        ( NULL != pucData ) &&
        ( SMBUS_STATUS_OK == iClaimSlave( ucAddr, bPec ) ) )
    {
        int iTempByte = i2c_smbus_read_byte( iBusHandle );
        
        if( LINUX_STATUS_RW_ERROR != iTempByte )
        {
            *pucData = ( uint8_t )iTempByte;
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

    if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) &&
        ( SMBUS_STATUS_OK == iClaimSlave( ucAddr, bPec ) ) )
    {
        if( LINUX_STATUS_RW_OK == i2c_smbus_write_byte_data( iBusHandle, ucCmd, ucData ) )
        {
            iStatus = SMBUS_STATUS_OK;
        }
    }

    return iStatus;
}

/**
 * @brief Call SMBus command "Read byte"
 */
int smbus_read_byte( uint8_t ucAddr, uint8_t ucCmd, uint8_t* pucData, bool bPec )
{
    int iStatus = SMBUS_STATUS_ERROR;

    if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) && 
        ( NULL != pucData ) &&
        ( SMBUS_STATUS_OK == iClaimSlave( ucAddr, bPec ) ) )
    {
        int iTempByte = i2c_smbus_read_byte_data( iBusHandle, ucCmd );
        
        if( LINUX_STATUS_RW_ERROR != iTempByte )
        {
            *pucData = ( uint8_t )iTempByte;
            iStatus = SMBUS_STATUS_OK;
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

    if( ( LINUX_STATUS_INVALID_HANDLE != iBusHandle ) &&
        ( SMBUS_STATUS_OK == iClaimSlave( ucAddr, bPec ) ) )
    {
        if( LINUX_STATUS_RW_OK == i2c_smbus_write_word_data( iBusHandle, ucCmd, usData ) )
        {
            iStatus = SMBUS_STATUS_OK;
        }
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
        ( NULL != pusData ) &&
        ( SMBUS_STATUS_OK == iClaimSlave( ucAddr, bPec ) ) )
    {
        int iTempWord = i2c_smbus_read_word_data( iBusHandle, ucCmd );

        if( LINUX_STATUS_RW_ERROR != iTempWord )
        {
            *pusData = ( uint16_t )iTempWord;
            iStatus = SMBUS_STATUS_OK;
        }
    }
    
    return iStatus;
}
