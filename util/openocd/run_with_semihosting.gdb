# GDB script to run program with semihosting
target extended-remote :3333
monitor arm semihosting enable
monitor reset halt
load
monitor reset init
continue
