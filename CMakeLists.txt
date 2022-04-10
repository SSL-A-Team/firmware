cmake_minimum_required(VERSION 3.22)

include(${CMAKE_SOURCE_DIR}/cmake/stm32-defines.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/header-utils.cmake)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/stm32-toolchain.cmake)

project(ATeam-Firmware
	LANGUAGES C CXX ASM)

macro(tests dir linker fname tcomment prefix)
	set(TEST_DIR ${dir})

	file(GLOB_RECURSE TEST_ASM_FILES "${TEST_DIR}/*.s")
	file(GLOB_RECURSE TEST_C_FILES "${TEST_DIR}/*.c")
	file(GLOB_RECURSE TEST_CXX_FILES "${TEST_DIR}/*.cpp")

	add_executable(${fname}.elf
		${TEST_ASM_FILES}
		${TEST_C_FILES}
		${TEST_CXX_FILES}
	)

	header_directories(TEST_INCLUDE_DIRS ${TEST_DIR})
	target_include_directories(${fname}.elf PRIVATE
		${TEST_INCLUDE_DIRS}
	)

	target_compile_definitions(${fname}.elf PRIVATE
		${${prefix}_DEFINITIONS}
	)

	target_compile_options(${fname}.elf PRIVATE
		$<$<COMPILE_LANGUAGE:C>:${${prefix}_C_OPTIONS}>
		$<$<COMPILE_LANGUAGE:CXX>:${${prefix}_CXX_OPTIONS}>
	) 

	set(test_linker_file "${CMAKE_CURRENT_SOURCE_DIR}/${TEST_DIR}/${linker}")
	target_link_options(${fname}.elf PRIVATE ${${prefix}_LINKER_OPTIONS} -T ${test_linker_file})
		
	#set_target_properties(test-breadboard-radio.elf PROPERTIES
	#	LINK_DEPENDS ${bb_radio_linker_file})

	add_custom_target(${fname}
		arm-none-eabi-objcopy -Obinary "${fname}.elf" "${fname}.bin"
		WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
		DEPENDS ${fname}.elf
		COMMENT ${tcomment}
	)

	add_custom_target(${fname}-prog
		openocd -f ${${prefix}_OPENOCD_CFG} -c "program bin/${fname}.bin verify reset exit ${STM32_FLASH_START_ADDR}"
	)
endmacro()

#######################################
#  Standalone Breadbaord Radio Tests  #
#######################################

# set(BB_RADIO_DIR "test/breadboard-radio/cube_ide")

# file(GLOB_RECURSE BB_RADIO_ASM_FILES "${BB_RADIO_DIR}/*.s")
# file(GLOB_RECURSE BB_RADIO_C_FILES "${BB_RADIO_DIR}/*.c")
# file(GLOB_RECURSE BB_RADIO_CXX_FILES "${BB_RADIO_DIR}/*.cpp")

# add_executable(test-breadboard-radio.elf
# 	${BB_RADIO_ASM_FILES}
# 	${BB_RADIO_C_FILES}
# 	${BB_RADIO_CXX_FILES}
# )

# header_directories(BB_RADIO_INCLUDE_DIRS ${BB_RADIO_DIR})
# target_include_directories(test-breadboard-radio.elf PRIVATE
# 	${BB_RADIO_INCLUDE_DIRS}
# )

# target_compile_definitions(test-breadboard-radio.elf PRIVATE
# 	${NUCLEO_F429ZI_DEFINITIONS}
# )

# target_compile_options(test-breadboard-radio.elf PRIVATE
# 	$<$<COMPILE_LANGUAGE:C>:${NUCLEO_F429ZI_C_OPTIONS}>
# 	$<$<COMPILE_LANGUAGE:CXX>:${NUCLEO_F429ZI_CXX_OPTIONS}>
# ) 

# set(bb_radio_linker_file "${CMAKE_CURRENT_SOURCE_DIR}/${BB_RADIO_DIR}/STM32F429ZITX_FLASH.ld")
# target_link_options(test-breadboard-radio.elf PRIVATE ${NUCLEO_F429ZI_LINKER_OPTIONS} -T ${bb_radio_linker_file})
	
# #set_target_properties(test-breadboard-radio.elf PROPERTIES
# #	LINK_DEPENDS ${bb_radio_linker_file})

# add_custom_target(test-breadboard-radio
# 	arm-none-eabi-objcopy -Obinary "test-breadboard-radio.elf" "test-breadboard-radio.bin"
# 	WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
# 	DEPENDS test-breadboard-radio.elf
# 	COMMENT "create flat binary for programming"
# )

# add_custom_target(test-breadboard-radio-prog
# 	openocd -f ${NUCLEO_F429ZI_OPENOCD_CFG} -c "program bin/test-breadboard-radio.bin verify reset exit ${STM32_FLASH_START_ADDR}"
# )

tests("test/breadboard-radio/cube_ide" "STM32F429ZITX_FLASH.ld" "test-breadboard-radio" "create flat binary for programming" "NUCLEO_F429ZI")
