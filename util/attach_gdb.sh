#!/bin/bash

if [ "$#" -ne 2 ]; then
	echo "Usage: attach.gdb <openocd config file> <target elf>"
	exit 1
fi

# TODO wait to confirm connected to target using something intelligent...
openocd -f $1 > /dev/null 2>&1 &
echo "launched openocd in the background"
sleep 1

arm-none-eabi-gdb \
	-ex "set confirm off" \
	-ex "target remote localhost:3333" \
	-ex "file $2" \
	-ex "load" -ex \
	-ex "set confirm on" \
	-ex "b main"

kill %1
