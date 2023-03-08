/**
 * @file test_utils.h
 * @author Konrad Mosoczy (konradm@amd.com)
 * @brief Utility functions for testing I2C/SMBus master/slave functionality
 * @date 2023-03-08
 * 
 * Copyright (c) 2023-present Advanced Micro Devices, Inc. All rights reserved.
 * 
 */

#ifndef SMBUS_TEST_UTILS_H
#define SMBUS_TEST_UTILS_H

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/

#define UNUSED( x )         ( void )( x )

#define BUFFER_PRINT_WIDTH  ( 8 )
#define SMBUS_PRINTF        { struct timespec T = { 0 }; clock_gettime( CLOCK_REALTIME, &T ); \
                            struct tm *t = localtime( &T.tv_sec );                            \
                            printf( "[ %02d:%02d:%02d:%03ld ] ",                              \
                                ( t->tm_hour ),                                               \
                                ( t->tm_min ),                                                \
                                ( t->tm_sec ),                                                \
                                ( T.tv_nsec/1000000 ) ); }                                    \
                            printf

#define PRINT_ENUM( x )    SMBUS_PRINTF( "%-40s: %d\r\n", #x, ( char )x )
#define SMBUS_PRINTF_DBG   SMBUS_PRINTF( "%d - ", __LINE__ ); printf
#define SMBUS_PRINTF_ERR   SMBUS_PRINTF_DBG( "ERROR - " ); printf
#define NEW_LINE           printf( "\r\n" );

/*****************************************************************************/
/* Function Declarations                                                     */
/*****************************************************************************/

/**
 * @brief Displays data buffer
 *
 * @param pucBuffer Buffer to display
 * @param usLength Number of bytes in buffer
 *
 * @return None
 */
void vDisplayBuffer( uint8_t *pucBuffer, uint16_t usLength );

/**
 * @brief Read a byte from stdin
 *
 * @param sPrompt Prompt to display to user
 * @param bHex TRUE if expected input value is b16, otherwise it will be b10
 * @param bNewLine TRUE if a newline should ne printed after the prompt
 *
 * @return Single byte, read from stdin (0 is the default value if there are errors)
 */
uint8_t ucInputByte( const char *sPrompt, bool bHex, bool bNewLine );

#endif
