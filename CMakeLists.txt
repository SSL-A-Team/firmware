cmake_minimum_required(VERSION 3.22)

project(ATeam-Firmware)

include(cmake/stm32-defines.cmake)

#######################################
#  Standalone Breadbaord Radio Tests  #
#######################################

file(GLOB_RECURSE BB_RADIO_ASM_FILES "test/nucleo_odinw2/cube_ide/*.s")
file(GLOB_RECURSE BB_RADIO_C_FILES "test/nucleo_odinw2/cube_ide/*.c")
file(GLOB_RECURSE BB_RADIO_CXX_FILES "test/nucleo_odinw2/cube_ide/*.cpp")

add_executable(test-breadboard-radio.elf
	${BB_RADIO_ASM_FILES}
	${BB_RADIO_C_FILES}
	${BB_RADIO_CXX_FILES}
)

file(GLOB_RECURSE BB_RADIO_INCLUDE_FILES "test/nucleo_odinw2/cube_ide/*.h")

target_include_directories(test-breadboard-radio.elf PRIVATE
	${BB_RADIO_INCLUDE_FILES}
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


