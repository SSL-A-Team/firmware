set(STM32_FLASH_START_ADDR 0x08000000)

###################
#  Nucleo F429ZI  #
###################

set(NUCLEO_F429ZI_MACHINE_OPTIONS -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard CACHE STRING "")
set(NUCLEO_F429ZI_C_OPTIONS ${NUCLEO_F429ZI_MACHINE_OPTIONS})
set(NUCLEO_F429ZI_CXX_OPTIONS ${NUCLEO_F429ZI_MACHINE_OPTIONS} -Wno-register -fno-exceptions -fno-rtti CACHE STRING "")
set(NUCLEO_F429ZI_LINKER_OPTIONS ${NUCLEO_F429ZI_MACHINE_OPTIONS} --specs=nosys.specs --specs=nano.specs -u _printf_float CACHE STRING "")
set(NUCLEO_F429ZI_DEFINITIONS -DSTM32F429xx)
set(NUCLEO_F429ZI_CHIP STM32F429ZI CACHE STRING "Full STM32 Chip Model")
set(NUCLEO_F429ZI_OPENOCD_CFG "board/stm32f429discovery.cfg")

#################
#  stspin32f0x  #
#################

set(STSPIN32F0x_MACHINE_OPTIONS -mthumb -mcpu=cortex-m0 -mfloat-abi=soft CACHE STRING "")
set(STSPIN32F0x_C_OPTIONS ${STSPIN32F0x_MACHINE_OPTIONS})
set(STSPIN32F0x_CXX_OPTIONS ${STSPIN32F0x_MACHINE_OPTIONS} -Wno-register -fno-exceptions -fno-rtti CACHE STRING "")
set(STSPIN32F0x_LINKER_OPTIONS ${STSPIN32F0x_MACHINE_OPTIONS} --specs=nosys.specs --specs=nano.specs CACHE STRING "")
set(STSPIN32F0x_DEFINITIONS -DSTM32F031xx)
set(STSPIN32F0x_CHIP STM32F031C6 CACHE STRING "Full STM32 Chip Model")
set(STSPIN32F0x_OPENOCD_CFG "board/stm32f0discovery.cfg")
