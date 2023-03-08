#!/bin/bash

ME=`basename "$0"`
BUS_NUM="10"
BUS="/sys/bus/i2c/devices/i2c-${BUS_NUM}"
DEVICE_NAME="i2c-slave-simple"
ADDR_PREFIX="0x"

if [ $# -eq 0 ]; then
    echo "Usage: ${ME} command [args...]"
    echo "       ${ME} ls"
    echo "       ${ME} mk 0x64"
    echo "       ${ME} rm 0x64"
    echo "       ${ME} dd-enable"
    echo "       ${ME} dd-disable"
    exit
fi

echo "Using bus ${BUS_NUM} at ${BUS}"

if [ "$1" == "ls" ] && [ $# -eq 1 ]; then
    ls ${BUS} | grep ${BUS_NUM}-
elif [ "$1" == "mk" ] && [ $# -eq 2 ]; then
    addr="${2}"
    slave=${addr#"$ADDR_PREFIX"}
    echo "Adding slave at address ${slave}"
    echo "${DEVICE_NAME} 0x10${slave}" > ${BUS}/new_device
elif [ "$1" == "rm" ] && [ $# -eq 2 ]; then
    addr="${2}"
    slave=${addr#"$ADDR_PREFIX"}
    echo "Removing slave at address ${slave}"
    echo "0x10${slave}" > ${BUS}/delete_device
elif [ "$1" == "dd-enable" ] && [ $# -eq 1 ]; then
    echo "Enabling debug for i2c-aspeed.c"
    echo "file i2c-aspeed.c line 0- +p" > /proc/dynamic_debug/control
elif [ "$1" == "dd-disable" ] && [ $# -eq 1 ]; then
    echo "Disabling debug for i2c-aspeed.c"
    echo "file i2c-aspeed.c line 0- -p" > /proc/dynamic_debug/control
else
    echo "Incorrect arguments"
    exit
fi

