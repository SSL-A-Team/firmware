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
set(STSPIN32F0x_C_OPTIONS ${STSPIN32F0x_MACHINE_OPTIONS} -Wall)
set(STSPIN32F0x_CXX_OPTIONS ${STSPIN32F0x_MACHINE_OPTIONS} -Wno-register -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections CACHE STRING "")
set(STSPIN32F0x_LINKER_OPTIONS ${STSPIN32F0x_MACHINE_OPTIONS} -Wl,--gc-sections,--no-warn-rwx-segments --specs=nosys.specs --specs=nano.specs CACHE STRING "")
set(STSPIN32F0x_DEFINITIONS -DSTM32F031xx)
set(STSPIN32F0x_CHIP STM32F031C6 CACHE STRING "Full STM32 Chip Model")
# set(STSPIN32F0x_OPENOCD_CFG "board/stm32f0discovery.cfg")
set(STSPIN32F0x_OPENOCD_CFG "${CMAKE_SOURCE_DIR}/../util/openocd/steval-spin320x.cfg")

###################
#  Nucleo G071RB  #
###################

set(NUCLEO_G071RB_MACHINE_OPTIONS -mthumb -mcpu=cortex-m0plus -mfloat-abi=soft CACHE STRING "")
set(NUCLEO_G071RB_C_OPTIONS ${NUCLEO_G071RB_MACHINE_OPTIONS} -Wall)
set(NUCLEO_G071RB_CXX_OPTIONS ${NUCLEO_G071RB_MACHINE_OPTIONS} -Wno-register -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections CACHE STRING "")
set(NUCLEO_G071RB_LINKER_OPTIONS ${NUCLEO_G071RB_MACHINE_OPTIONS} -Wl,--gc-sections --specs=rdimon.specs -lrdimon CACHE STRING "")
set(NUCLEO_G071RB_DEFINITIONS -DSTM32G071xx)
set(NUCLEO_G071RB_CHIP STM32G071RB CACHE STRING "Full STM32 Chip Model")
set(NUCLEO_G071RB_OPENOCD_CFG "${CMAKE_SOURCE_DIR}/../util/openocd/nucleo-g071rb.cfg")