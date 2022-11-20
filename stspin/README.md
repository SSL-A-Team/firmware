# StSpin Firmware

## Debugging Binary Size Issues

The StSpin has a small flash capacity. It may become necessary to find the sources of an explosion in binary size.

1. temporarily increase the FLASH size in stm32f031xx.ld
2. run `nm --print-size --size-sort --radix=d steval-spin3201-6step.elf` on the elf output

Double precision promotion is coerced into a hard error as the arith function take about 7k of flash.