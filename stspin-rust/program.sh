#!/bin/bash

OPENOCD_CONFIG="board/stm32f0discovery.cfg"

BINARY=$1

openocd -f $OPENOCD_CONFIG -c "program $BINARY reset exit"
if [ $? != 0 ]; then
    echo "Error programming board"
    exit 1
fi

if [ "$2" == "monitor" ]; then
    openocd -f "$OPENOCD_CONFIG" -c "init; arm semihosting enable; reset"
fi