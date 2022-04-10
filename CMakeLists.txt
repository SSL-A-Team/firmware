cmake_minimum_required(VERSION 3.22)

include(${CMAKE_SOURCE_DIR}/cmake/stm32-defines.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/header-utils.cmake)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/stm32-toolchain.cmake)

project(ATeam-Firmware
	LANGUAGES C CXX ASM)

#######################################
#  Standalone Breadbaord Radio Tests  #
#######################################

set(BB_RADIO_DIR "test/breadboard-radio/cube_ide")

file(GLOB_RECURSE BB_RADIO_ASM_FILES "${BB_RADIO_DIR}/*.s")
file(GLOB_RECURSE BB_RADIO_C_FILES "${BB_RADIO_DIR}/*.c")
file(GLOB_RECURSE BB_RADIO_CXX_FILES "${BB_RADIO_DIR}/*.cpp")

add_executable(test-breadboard-radio.elf
	${BB_RADIO_ASM_FILES}
	${BB_RADIO_C_FILES}
	${BB_RADIO_CXX_FILES}
)

header_directories(BB_RADIO_INCLUDE_DIRS ${BB_RADIO_DIR})
target_include_directories(test-breadboard-radio.elf PRIVATE
	${BB_RADIO_INCLUDE_DIRS}
)

target_compile_definitions(test-breadboard-radio.elf PRIVATE
	${NUCLEO_F429ZI_DEFINITIONS}
)

target_compile_options(test-breadboard-radio.elf PRIVATE
	$<$<COMPILE_LANGUAGE:C>:${NUCLEO_F429ZI_C_OPTIONS}>
	$<$<COMPILE_LANGUAGE:CXX>:${NUCLEO_F429ZI_CXX_OPTIONS}>
) 

set(bb_radio_linker_file "${CMAKE_CURRENT_SOURCE_DIR}/${BB_RADIO_DIR}/STM32F429ZITX_FLASH.ld")
target_link_options(test-breadboard-radio.elf PRIVATE ${NUCLEO_F429ZI_LINKER_OPTIONS} -T ${bb_radio_linker_file})
	
#set_target_properties(test-breadboard-radio.elf PROPERTIES
#	LINK_DEPENDS ${bb_radio_linker_file})

add_custom_target(test-breadboard-radio
	arm-none-eabi-objcopy -Obinary "test-breadboard-radio.elf" "test-breadboard-radio.bin"
	WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
	DEPENDS test-breadboard-radio.elf
	COMMENT "create flat binary for programming"
)

add_custom_target(test-breadboard-radio-prog
	openocd -f ${NUCLEO_F429ZI_OPENOCD_CFG} -c "program bin/test-breadboard-radio.bin verify reset exit ${STM32_FLASH_START_ADDR}"
)


#####################
#  STEVAL-SPIN3201  #
#####################

set(STEVAL_SPIN3201_DIR "motor")

file(GLOB_RECURSE STEVAL_SPIN3201_ASM_FILES "${STEVAL_SPIN3201_DIR}/*.s")
file(GLOB_RECURSE STEVAL_SPIN3201_C_FILES "${STEVAL_SPIN3201_DIR}/*.c")
file(GLOB_RECURSE STEVAL_SPIN3201_CXX_FILES "${STEVAL_SPIN3201_DIR}/*.cpp")

add_executable(steval-spin3201-6step.elf
	${STEVAL_SPIN3201_ASM_FILES}
	${STEVAL_SPIN3201_C_FILES}
	${STEVAL_SPIN3201_CXX_FILES}
)

header_directories(STEVAL_SPIN3201_INCLUDE_DIRS ${STEVAL_SPIN3201_DIR})
target_include_directories(steval-spin3201-6step.elf PRIVATE
	${STEVAL_SPIN3201_INCLUDE_DIRS}
)

target_compile_definitions(steval-spin3201-6step.elf PRIVATE
	${STSPIN32F0x_DEFINITIONS}
)

target_compile_options(steval-spin3201-6step.elf PRIVATE
	$<$<COMPILE_LANGUAGE:C>:${STSPIN32F0x_C_OPTIONS}>
	$<$<COMPILE_LANGUAGE:CXX>:${STSPIN32F0x_CXX_OPTIONS}>
) 

set(stspin_eval3201_linker_file "${CMAKE_CURRENT_SOURCE_DIR}/${STEVAL_SPIN3201_DIR}/common/stm32f031xx.ld")
target_link_options(steval-spin3201-6step.elf PRIVATE ${STSPIN32F0x_LINKER_OPTIONS} -T ${stspin_eval3201_linker_file})
	
#set_target_properties(test-breadboard-radio.elf PROPERTIES
#	LINK_DEPENDS ${bb_radio_linker_file})

add_custom_target(steval-spin3201-6step
	arm-none-eabi-objcopy -Obinary "steval-spin3201-6step.elf" "steval-spin3201-6step.bin"
	WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
	DEPENDS steval-spin3201-6step.elf
	COMMENT "create flat binary for programming"
)

add_custom_target(steval-spin3201-6step-prog
	openocd -f ${STSPIN32F0x_OPENOCD_CFG} -c "program bin/steval-spin3201-6step.bin verify reset exit ${STM32_FLASH_START_ADDR}"
)


