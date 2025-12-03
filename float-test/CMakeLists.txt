cmake_minimum_required(VERSION 3.22)

include(${CMAKE_SOURCE_DIR}/../util/cmake/stm32-defines.cmake)
include(${CMAKE_SOURCE_DIR}/../util/cmake/stm32-etarget.cmake)
include(${CMAKE_SOURCE_DIR}/../util/cmake/header-utils.cmake)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/../util/cmake/stm32-toolchain.cmake)

project(ATeam-FloatTest
	LANGUAGES C CXX ASM)

set(CMAKE_BUILD_TYPE Debug)

#########################
#  Float Test Project   #
#  Target: Nucleo-G071RB #
#########################

set(FLOAT_TEST_DIR "${CMAKE_SOURCE_DIR}")
set(FLOAT_TEST_LINKER_FILE "${FLOAT_TEST_DIR}/common/stm32g071xx.ld")
set(FLOAT_TEST_PREFIX "NUCLEO_G071RB")
ateam_add_targets(${FLOAT_TEST_DIR} ${FLOAT_TEST_LINKER_FILE} ${FLOAT_TEST_PREFIX})
