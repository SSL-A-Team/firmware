#!/bin/bash

OPENOCD_CONFIG="board/st_nucleo_h743zi.cfg"

BINARY=$1

openocd -f $OPENOCD_CONFIG -d1 -c "program $BINARY reset exit"
if [ $? != 0 ]; then
    echo "Error programming board"
    exit 1
fi

if [ "$2" == "monitor" ]; then
    openocd -f "$OPENOCD_CONFIG" -d1 -c "init; arm semihosting enable; reset"
    # arm-none-eabi-gdb 
fi