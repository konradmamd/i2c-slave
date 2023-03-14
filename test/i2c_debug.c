/**
 * @file i2c_debug.c
 * @author Konrad Mosoczy (konradm@amd.com)
 * @brief Source code for the `i2c-debug` tool similar to `i2c-tools`
 * @date 2022-12-21
 * 
 * Copyright (c) 2022-present Advanced Micro Devices, Inc. All rights reserved.
 * 
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "smbus.h"

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/**
 * @enum I2C_DEBUG_ARGC
 * @brief Argument counts for different commands.
 */
typedef enum I2C_DEBUG_ARGC
{
    I2C_DEBUG_ARGC_MIN              = 2,  /* min arguments to binary */
    I2C_DEBUG_DEV_ARGC_MIN          = 2,  /* min arguments in device string */
    I2C_DEBUG_DEV_ARGC_MAX          = 3,  /* max arguments in device string */
    I2C_DEBUG_DEV_ARGC_MAX_FLAGS    = 2,  /* max flags */
    I2C_DEBUG_WB_ARGC_MIN           = 4,  /* write byte */
    I2C_DEBUG_WW_ARGC_MIN           = 4,  /* write word */
    I2C_DEBUG_WBLOCK_ARGC_MIN       = 5,  /* write block */
    I2C_DEBUG_W32_ARGC_MIN          = 4,  /* write 32 */
    I2C_DEBUG_W64_ARGC_MIN          = 4,  /* write 64 */
    I2C_DEBUG_RB_ARGC_MIN           = 3,  /* read byte */
    I2C_DEBUG_RW_ARGC_MIN           = 3,  /* read word */
    I2C_DEBUG_RBLOCK_ARGC_MIN       = 3,  /* read block */
    I2C_DEBUG_R32_ARGC_MIN          = 3,  /* read 32 */
    I2C_DEBUG_R64_ARGC_MIN          = 3,  /* read 64 */
    I2C_DEBUG_PW_ARGC_MIN           = 4,  /* process call */
    I2C_DEBUG_PB_ARGC_MIN           = 5,  /* block process call */

} I2C_DEBUG_ARGC;

/**
 * @enum I2C_DEBUG_ARGV
 * @brief Argument positions for different commands.
 */
typedef enum I2C_DEBUG_ARGV
{
    I2C_DEBUG_ARGV_PROG          = 0,  /* binary name */
    I2C_DEBUG_ARGV_DEV           = 1,  /* device string */
    I2C_DEBUG_ARGV_CMD           = 2,  /* command */

    I2C_DEBUG_ARGV_WB_CMD        = 3,  /* write byte : command code */
    I2C_DEBUG_ARGV_WB_BYTE       = 4,  /* write byte : byte to write */

    I2C_DEBUG_ARGV_WW_CMD        = 3,  /* write word : command code */
    I2C_DEBUG_ARGV_WW_WORD       = 4,  /* write word : word to write */

    I2C_DEBUG_ARGV_WBLOCK_CMD    = 3,  /* write block : command code */
    I2C_DEBUG_ARGV_WBLOCK_SIZE   = 4,  /* write block : block size */

    I2C_DEBUG_ARGV_W32_CMD       = 3,  /* write 32: command code */
    I2C_DEBUG_ARGV_W32_DATA      = 4,  /* write 32: data to write */

    I2C_DEBUG_ARGV_W64_CMD       = 3,  /* write 64: command code */
    I2C_DEBUG_ARGV_W64_DATA      = 4,  /* write 64: data to write */

    I2C_DEBUG_ARGV_RB_CMD        = 3,  /* read byte : command code */

    I2C_DEBUG_ARGV_RW_CMD        = 3,  /* read word : command code */

    I2C_DEBUG_ARGV_RBLOCK_CMD    = 3,  /* read block : command code */

    I2C_DEBUG_ARGV_R32_CMD       = 3,  /* read 32: command code */

    I2C_DEBUG_ARGV_R64_CMD       = 3,  /* read 64: command code */

    I2C_DEBUG_ARGV_PW_CMD        = 3,  /* proc call : command code */
    I2C_DEBUG_ARGV_PW_WORD       = 4,  /* proc call : word to write */

    I2C_DEBUG_ARGV_PB_CMD        = 3,  /* block proc call : command code */
    I2C_DEBUG_ARGV_PB_SIZE       = 4,  /* block proc call : block size */

} I2C_DEBUG_ARGV;

/**
 * @enum I2C_DEBUG_FLAGS
 * @brief Flags to use when opening the I2C device (e.g. PEC).
 */
typedef enum I2C_DEBUG_FLAGS
{
    I2C_DEBUG_FLAGS_PEC           = 'p',
    I2C_DEBUG_FLAGS_DISABLE_RETRY = 'R'

} I2C_DEBUG_FLAGS;

/*****************************************************************************/
/* Function defintions                                                       */
/*****************************************************************************/

/**
 * @brief Main entry point of debug utility.
 */
int main( int argc, char* argv[] )
{
    if( I2C_DEBUG_ARGC_MIN > argc )
    {
        fprintf( 
            stderr, 
            "Usage: %s <addr>@<bus>[:pR] [cmd] [args...]\r\n\r\n"
            "Available Commands:\r\n"
            "   wb: write byte <command> <data>\r\n"
            "   ww: write word <command> <data>\r\n"
            "   wB: write block <command> <len> [<byte1> <byte2>...]\r\n"
            "  w32: write 32 <command> <data>\r\n"
            "  w64: write 64 <command> <data>\r\n"
            "   rb: read byte <command>\r\n"
            "   rw: read word <command>\r\n"
            "   rB: read block <command> <len>\r\n"
            "  r32: read 32 <command>\r\n"
            "  r64: read 64 <command>\r\n"
            "   pw: process call <command> <data>\r\n"
            "   pB: block process call <command> <len> [<byte1> <byte2>...]\r\n",
            argv[0]
        );

        exit( EXIT_FAILURE );
    }

    uint8_t ucAddr = 0;
    int iBus = 0;
    bool bPecEnabled = false;
    bool bDisableRetries = false;  /* TODO: Remove or rework this with new API. */
    char pcFlags[ I2C_DEBUG_DEV_ARGC_MAX_FLAGS + 1 ] = { 0 };

    int iRead = sscanf( argv[ I2C_DEBUG_ARGV_DEV ], "0x%02hhX@%d:%s", &ucAddr, &iBus, pcFlags );

    if( I2C_DEBUG_DEV_ARGC_MIN > iRead )
    {
        fprintf( stderr, "ERROR: Incorrect arguments.\r\n" );
        exit( EXIT_FAILURE );
    }

    if( I2C_DEBUG_DEV_ARGC_MAX == iRead )
    {
        if( NULL != strchr( pcFlags, I2C_DEBUG_FLAGS_PEC ) )
        {
            bPecEnabled = true;
        }

        if( NULL != strchr( pcFlags, I2C_DEBUG_FLAGS_DISABLE_RETRY ) )
        {
            bDisableRetries = true;
        }
    }

    if( SMBUS_STATUS_OK == smbus_init( iBus, SMBUS_MIN_BITRATE_KHZ ) )
    {
        printf( 
            "[*] Opened device at address 0x%02hhX on bus %d with flags { p=%d R=%d }.\r\n",
            ucAddr, iBus, bPecEnabled, bDisableRetries
        );
    }
    else
    {
        fprintf( stderr, "ERROR: Could not open device!\r\n" );
        exit( EXIT_FAILURE );
    }

    if( I2C_DEBUG_ARGC_MIN < argc )
    {
        char* pcUserInput = argv[ I2C_DEBUG_ARGV_CMD ];

        if( 0 == strcmp( "wb", pcUserInput ) )
        {
            if( I2C_DEBUG_WB_ARGC_MIN < argc )  /* write byte <command> <data> */
            {
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_WB_CMD ], NULL, 0 );
                uint8_t ucByte = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_WB_BYTE ], NULL, 0 );

                if( SMBUS_STATUS_OK == smbus_write_byte( ucAddr, ucCommand, ucByte, bPecEnabled ) )
                {
                    printf( "Wrote byte 0x%02hhX with command 0x%02X\r\n", ucByte, ucCommand );
                }
                else
                {
                    printf( "Could not write byte.\r\n" );
                }
            }
            else
            {
                fprintf( stderr, "ERROR: Specify a command and data byte.\r\n" );
            }
        }
        else if( 0 == strcmp( "ww", pcUserInput ) )
        {
            if( I2C_DEBUG_WW_ARGC_MIN < argc )  /* write word <command> <data> */
            {
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_WW_CMD ], NULL, 0 );
                uint16_t usWord = ( uint16_t )strtol( argv[ I2C_DEBUG_ARGV_WW_WORD ], NULL, 0 );

                if( SMBUS_STATUS_OK == smbus_write_word( ucAddr, ucCommand, usWord, bPecEnabled ) )
                {
                    printf( "Wrote word 0x%04X with command 0x%02X\r\n", usWord, ucCommand );
                }
                else
                {
                    printf( "Could not write word.\r\n" );
                }
            }
            else
            {
                fprintf( stderr, "ERROR: Specify a command and data word.\r\n" );
            }
        }
        else if( 0 == strcmp( "wB", pcUserInput ) )
        {
            if( I2C_DEBUG_WBLOCK_ARGC_MIN < argc ) /* write block <command> <len> [<byte1> <byte2>...] */
            {
                uint8_t pucBuffer[ SMBUS_MAX_DATA ] = { 0 };
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_WBLOCK_CMD ], NULL, 0 );
                uint8_t ucBlockSize = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_WBLOCK_SIZE ], NULL, 0 );

                if( ( SMBUS_MAX_DATA >= ucBlockSize ) && ( 0 < ucBlockSize ) && 
                    ( ( argc - I2C_DEBUG_WBLOCK_ARGC_MIN ) == ucBlockSize) )
                {
                    /* Push provided data bytes into buffer. */
                    int i;
                    for( i = 0; i < ucBlockSize; i++ )
                    {
                        pucBuffer[ i ] = ( uint8_t )strtol( argv[ i + I2C_DEBUG_WBLOCK_ARGC_MIN ], NULL, 0 );
                    }
                    
                    if( SMBUS_STATUS_OK == smbus_block_write( ucAddr, ucCommand, pucBuffer, ucBlockSize, bPecEnabled ) )
                    {
                        printf( "%d bytes successfully written with command 0x%02X\r\n0x", ucBlockSize, ucCommand );

                        for( i = 0; i < ucBlockSize; i++ )
                        {
                            printf( "%02X", pucBuffer[ i ] );
                        }

                        printf( "\r\n" );
                    }
                    else
                    {
                        printf( "Could not write block.\r\n" );
                    }
                }
                else
                {
                    fprintf( stderr, "ERROR: Incorrect block size.\r\n");
                }
            }
            else 
            {
                fprintf( stderr, "ERROR: Specify a command, block size, and data bytes.\r\n" );
            }
        }
        else if( 0 == strcmp( "w32", pcUserInput ) )
        {
            if( I2C_DEBUG_W32_ARGC_MIN < argc )  /* write 32 <command> <data> */
            {
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_W32_CMD ], NULL, 0 );
                uint32_t ulData = ( uint32_t )strtol( argv[ I2C_DEBUG_ARGV_W32_DATA ], NULL, 0 );

                if( SMBUS_STATUS_OK == smbus_write_32( ucAddr, ucCommand, ulData, bPecEnabled ) )
                {
                    printf( "Wrote data 0x%08X with command 0x%02X\r\n", ulData, ucCommand );
                }
                else
                {
                    printf( "Could not write data.\r\n" );
                }
            }
            else
            {
                fprintf( stderr, "ERROR: Specify a command and data.\r\n" );
            }
        }
        else if( 0 == strcmp ( "w64", pcUserInput ) )
        {
            if( I2C_DEBUG_W64_ARGC_MIN < argc )  /* write 64 <command> <data> */
            {
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_W64_CMD ], NULL, 0 );
                uint64_t ullData = ( uint64_t )strtol( argv[ I2C_DEBUG_ARGV_W64_DATA ], NULL, 0 );

                if( SMBUS_STATUS_OK == smbus_write_64( ucAddr, ucCommand, ullData, bPecEnabled ) )
                {
                    printf( "Wrote data 0x%016llX with command 0x%02X\r\n", ullData, ucCommand );
                }
                else
                {
                    printf( "Could not write data.\r\n" );
                }
            }
            else
            {
                fprintf( stderr, "ERROR: Specify a command and data.\r\n" );
            }
        }
        else if( 0 == strcmp( "rb", pcUserInput ) )
        {
            if( I2C_DEBUG_RB_ARGC_MIN < argc )  /* read byte <command> */
            {
                uint8_t ucByte = 0;
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_RB_CMD ], NULL, 0 );

                if( SMBUS_STATUS_OK == smbus_read_byte( ucAddr, ucCommand, &ucByte, bPecEnabled ) )
                {   
                    printf( "Read byte 0x%02X with command 0x%02X\r\n", ucByte, ucCommand );
                }
                else
                {
                    printf( "Could not read data.\r\n" );
                }
            }
            else 
            {
                fprintf( stderr, "ERROR: Specify a command.\r\n" );
            }
        }
        else if( 0 == strcmp( "rw", pcUserInput ) )
        {
            if( I2C_DEBUG_RW_ARGC_MIN < argc )  /* read word <command> */
            {
                uint16_t usWord = 0;
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_RW_CMD ], NULL, 0 );

                if( SMBUS_STATUS_OK == smbus_read_word( ucAddr, ucCommand, &usWord, bPecEnabled ) )
                {   
                    printf( "Read word 0x%04X with command 0x%02X\r\n", usWord, ucCommand );
                }
                else
                {
                    printf( "Could not read data.\r\n" );
                }
            }
            else 
            {
                fprintf( stderr, "ERROR: Specify a command.\r\n" );
            }
        }
        else if( 0 == strcmp( "rB", pcUserInput ) )
        {
            if( I2C_DEBUG_RBLOCK_ARGC_MIN < argc ) /* read block <command> */
            {
                uint8_t pucBuffer[ SMBUS_MAX_DATA ] = { 0 };
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_RBLOCK_CMD ], NULL, 0 );
                uint8_t ucBlockSize = 0;

                if( SMBUS_STATUS_OK == smbus_block_read( ucAddr, ucCommand, pucBuffer, &ucBlockSize, bPecEnabled ) )
                {
                    printf( "Read %d bytes with command 0x%02X\r\n0x", ucBlockSize, ucCommand );

                    int i;
                    for( i = 0; i < ucBlockSize; i++ )
                    {
                        printf( "%02X", pucBuffer[ i ] );
                    }

                    printf( "\r\n" );
                }
                else
                {
                    printf( "Could not read data.\r\n" );
                }
            }
            else 
            {
                fprintf( stderr, "ERROR: Specify a command and length to read.\r\n" );
            }
        }
        else if( 0 == strcmp( "r32", pcUserInput ) )
        {
            if( I2C_DEBUG_R32_ARGC_MIN < argc )  /* read 32 <command> */
            {
                uint32_t ulData = 0;
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_R32_CMD ], NULL, 0 );

                if( SMBUS_STATUS_OK == smbus_read_32( ucAddr, ucCommand, &ulData, bPecEnabled ) )
                {
                    printf( "Read data 0x%08X with command 0x%02X\r\n", ulData, ucCommand );
                }
                else
                {
                    printf( "Could not read data.\r\n" );
                }
            }
            else
            {
                fprintf( stderr, "ERROR: Specify a command.\r\n" );
            }
        }
        else if( 0 == strcmp ( "r64", pcUserInput ) )
        {
            if( I2C_DEBUG_R64_ARGC_MIN < argc )  /* read 64 <command> */
            {
                uint64_t ullData = 0;
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_R64_CMD ], NULL, 0 );

                if( SMBUS_STATUS_OK == smbus_read_64( ucAddr, ucCommand, &ullData, bPecEnabled ) )
                {
                    printf( "Read data 0x%016llX with command 0x%02X\r\n", ullData, ucCommand );
                }
                else
                {
                    printf( "Could not read data.\r\n" );
                }
            }
            else
            {
                fprintf( stderr, "ERROR: Specify a command.\r\n" );
            }
        }
        else if( 0 == strcmp( "pw", pcUserInput ) ) 
        {
            if( I2C_DEBUG_PW_ARGC_MIN < argc )  /* process call <command> <data> */
            {
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_PW_CMD ], NULL, 0 );
                uint16_t usWord = ( uint16_t )strtol( argv[ I2C_DEBUG_ARGV_PW_WORD ], NULL, 0 );
                uint16_t usWordWritten = 0;

                if( SMBUS_STATUS_OK == smbus_process_call( ucAddr, ucCommand, usWord, &usWordWritten, bPecEnabled ) )
                {
                    printf( "Wrote word 0x%04X with command 0x%02X\r\n", usWord, ucCommand );
                    printf( "Read back word 0x%04X\r\n", usWordWritten );
                }
                else
                {
                    printf( "Could not perform process call.\r\n" );
                }
            }
            else
            {
                fprintf( stderr, "ERROR: Specify a command and data word.\r\n" );
            }
        }
        else if( 0 == strcmp( "pB", pcUserInput ) )
        {
            if( I2C_DEBUG_PB_ARGC_MIN < argc )  /* block process call <command> <len> [<byte1> <byte2>...] */
            {
                uint8_t pucBuffer[ SMBUS_MAX_DATA ] = { 0 };
                uint8_t pucReadBuffer[ SMBUS_MAX_DATA ] = { 0 };
                uint8_t ucCommand = ( uint8_t )strtol( argv[ I2C_DEBUG_ARGV_PB_CMD ], NULL, 0 );
                uint8_t ucBlockSize = ( uint16_t )strtol( argv[ I2C_DEBUG_ARGV_PB_SIZE ], NULL, 0 );
                uint8_t ucReadBlockSize = 0;
                
                if( ( SMBUS_MAX_DATA >= ucBlockSize ) && ( 0 < ucBlockSize ) && 
                    ( ( argc - I2C_DEBUG_PB_ARGC_MIN ) == ucBlockSize) )
                {
                    /* Push provided data bytes into buffer. */
                    int i;
                    for( i = 0; i < ucBlockSize; i++ )
                    {
                        pucBuffer[ i ] = ( uint8_t )strtol( argv[ i + I2C_DEBUG_PB_ARGC_MIN ], NULL, 0 );
                    }

                    if( SMBUS_STATUS_OK == smbus_block_process_call( 
                        ucAddr, ucCommand, pucBuffer, ucBlockSize, pucReadBuffer, &ucReadBlockSize, bPecEnabled ) )
                    {
                        printf( "Read back %d bytes\r\n0x", ucReadBlockSize );

                        for( i = 0; i < ucReadBlockSize; i++ )
                        {
                            printf( "%02X", pucReadBuffer[ i ] );
                        }

                        printf( "\r\n" );
                    }
                    else
                    {
                        printf( "Could not perform block process call.\r\n" );
                    }
                }
                else
                {
                    fprintf( stderr, "ERROR: Incorrect block size.\r\n" );
                }
            }
            else
            {
                fprintf( stderr, "ERROR: Specify a command, block size, and data bytes.\r\n" );
            }
        }
        else
        {
            printf( "ERROR: Unknown command (%s)\r\n", pcUserInput );
        }
    }

    if( SMBUS_STATUS_OK == smbus_deinit() )
    {
        printf( "[*] Closed device.\r\n" );
    }
    else
    {
        fprintf( stderr, "ERROR: Could not close device!\r\n" );
        exit( EXIT_FAILURE );
    }
}
