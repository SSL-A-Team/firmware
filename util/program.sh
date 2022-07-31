#!/bin/bash

if [ "$#" -ne 2 || "$#" -ne 3 ]; then
	echo "Usage: program.sh <openocd config file> <target elf> opt:\"monitor\""
	exit 1
fi

OPENOCD_CONFIG=$1
BINARY=$2

openocd -f $OPENOCD_CONFIG -d1 -c "program $BINARY reset exit"
if [ $? != 0 ]; then
    echo "Error programming board"
    exit 1
fi

if [ "$3" == "monitor" ]; then
    openocd -f "$OPENOCD_CONFIG" -d1 -c "init; arm semihosting enable; reset"
fi