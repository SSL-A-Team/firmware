cmake_minimum_required(VERSION 3.22)

include(${CMAKE_SOURCE_DIR}/../util/cmake/stm32-defines.cmake)
include(${CMAKE_SOURCE_DIR}/../util/cmake/stm32-etarget.cmake)
include(${CMAKE_SOURCE_DIR}/../util/cmake/header-utils.cmake)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/../util/cmake/stm32-toolchain.cmake)

project(ATeam-Firmware
	LANGUAGES C CXX ASM)

set(CMAKE_BUILD_TYPE Debug)

#####################
#  STEVAL-SPIN3201  #
#####################

set(STEVAL_SPIN3201_DIR "${CMAKE_SOURCE_DIR}")
set(STEVAL_SPIN3201_LINKER_FILE "${STEVAL_SPIN3201_DIR}/common/stm32f031xx.ld")
set(STEVAL_SPIN3201_PREFIX "STSPIN32F0x")
ateam_add_targets(${STEVAL_SPIN3201_DIR} ${STEVAL_SPIN3201_LINKER_FILE} ${STEVAL_SPIN3201_PREFIX})

