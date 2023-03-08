/**
 * @file test_master.c
 * @author Konrad Mosoczy (konradm@amd.com)
 * @brief Test SMBus master transactions using Linux API
 * @date 2023-03-08
 * 
 * Copyright (c) 2023-present Advanced Micro Devices, Inc. All rights reserved.
 * 
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "smbus.h"

int main()
{
    uint16_t usData = 0;
    uint8_t ucBus = 10;
    uint8_t ucTargetAddr = 0x64;

    printf( "Reading word from target 0x%02X on bus %d\r\n", ucTargetAddr, ucBus );

    if( SMBUS_STATUS_OK == smbus_init( ucBus, 100 ) )
    {
        printf( "Initialized bus.\r\n" );

        if( SMBUS_STATUS_OK == smbus_read_word( ucTargetAddr, 0x08, &usData, false ) )
        {
            printf( "Read word 0x%04X\r\n", usData );
        }
        else
        {
            printf( "Could not read data.\r\n" );
        }

        smbus_deinit();
    }
    else
    {
        printf( "Could not initialize bus.\r\n" );
    }
}
