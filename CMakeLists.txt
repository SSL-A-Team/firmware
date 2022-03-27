cmake_minimum_required(VERSION 3.22)

include(${CMAKE_SOURCE_DIR}/cmake/stm32-defines.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/header-utils.cmake)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/stm32-toolchain.cmake)

project(ATeam-Firmware)

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

add_custom_target(test-breadboard-radio
	objcopy -Obinary "test-breadboard-radio.elf" "test-breadboard-radio.bin"
	WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
	DEPENDS test-breadboard-radio.elf
	COMMENT "create flat binary for programming"
)

add_custom_target(test-breadboard-radio-prog
	echo "TODO"
)


