###################
#  Nucleo F429ZI  #
###################

set(NUCLEO_F429ZI_MACHINE_OPTIONS "-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard" CACHE STRING "")
set(NUCLEO_F429ZI_C_OPTIONS "${NUCLEO_F429ZI_MACHINE_OPTIONS}")
set(NUCLEO_F429ZI_CXX_OPTIONS "${NUCLEO_F429_MACHINE_OPTIONS} -Wno-register -fno-exceptions -fno-rtti" CACHE STRING "")
set(NUCLEO_F429ZI_LINKER_OPTIONS "--specs=nosys.specs --specs=nano.specs -u _printf_float" CACHE STRING "")
set(NUCLEO_F429ZI_DEFINITIONS "-DSTM32F429XX")
set(NUCLEO_F429ZI_CHIP STM32F429ZI CACHE STRING "Full STM32 Chip Model")
