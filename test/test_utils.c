/**
 * @file test_utils.c
 * @author Konrad Mosoczy (konradm@amd.com)
 * @brief Utility functions for testing I2C/SMBus master/slave functionality
 * @date 2023-03-08
 * 
 * Copyright (c) 2023-present Advanced Micro Devices, Inc. All rights reserved.
 * 
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

#include "test_utils.h"

/*****************************************************************************/
/* Function Implementations                                                  */
/*****************************************************************************/

/**
 * @brief Read a byte from stdin
 */
uint8_t ucInputByte( const char *sPrompt, bool bHex, bool bNewLine )
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
void vDisplayBuffer( uint8_t *pucBuffer, uint16_t usLength )
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
