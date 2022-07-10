macro(VALIDATE_ARM_NONE_EABI_ROOT toolchain_root)
	message(STATUS "ARM NONE EABI Toolchain - validating root (${toolchain_root})")
	if (NOT EXISTS ${toolchain_root})
		message(FATAL_ERROR "ARM NONE EABI Toolchain - root does not exist")
	endif()

	set(root "${toolchain_root}")
	set(path_list "")
	cmake_path(APPEND root "bin" OUTPUT_VARIABLE toolchain_bin_dir)
	cmake_path(APPEND root "arm-none-eabi" "include" OUTPUT_VARIABLE toolchain_include_dir)
	cmake_path(APPEND root "arm-none-eabi" "lib" OUTPUT_VARIABLE toolchain_lib_dir)
	list(APPEND path_list ${toolchain_bin_dir} ${toolchain_include_dir} ${toolchain_lib_dir})

	# set(path_error FALSE)
	# foreach(toolchain_path ${path_list})
	# 	if (NOT EXISTS ${toolchain_path})
	# 		message(WARNING "ARM NONE EABI Toolchain - path does not exist (${toolchain_path})")
	# 		set(path_error TRUE)
	# 	else()
	# 		message(STATUS "ARM NONE EABI Toolchain - found path (${toolchain_path})")
	# 	endif()

	# endforeach()

	# if (${path_error})
	# 	message(FATAL_ERROR "ARM NONE EABI Toolchain - Tool validation failed. One or more tool path was missing.")
	# endif()

	set(bin_list "")
	cmake_path(APPEND toolchain_bin_dir "arm-none-eabi-as" OUTPUT_VARIABLE toolchain_assembler)
	cmake_path(APPEND toolchain_bin_dir "arm-none-eabi-gcc" OUTPUT_VARIABLE toolchain_c_compliler)
	cmake_path(APPEND toolchain_bin_dir "arm-none-eabi-g++" OUTPUT_VARIABLE toolchain_cxx_compiler)
	cmake_path(APPEND toolchain_bin_dir "arm-none-eabi-ld" OUTPUT_VARIABLE toolchain_linker)
	cmake_path(APPEND toolchain_bin_dir "arm-none-eabi-objcopy" OUTPUT_VARIABLE toolchain_objcopy)
	cmake_path(APPEND toolchain_bin_dir "arm-none-eabi-objdump" OUTPUT_VARIABLE toolchain_objdump)
	cmake_path(APPEND toolchain_bin_dir "arm-none-eabi-readelf" OUTPUT_VARIABLE toolchain_readelf)
	cmake_path(APPEND toolchain_bin_dir "arm-none-eabi-size" OUTPUT_VARIABLE toolchain_size)
	list(APPEND bin_list 
		${toolchain_assembler} 
		${toolchain_c_compliler} 
		${toolchain_cxx_compiler}
		${toolchain_linker}
		${toolchain_objcopy}
		${toolchain_objdump}
		${toolchain_readelf}
		${toolchain_size})

	foreach(tool ${bin_list})
		set(tool_error FALSE)
		if (NOT EXISTS ${tool})
			message(WARNING "ARM NONE EABI Toolchain - tool does not exist (${tool})")
			set(tool_error TRUE)
		else()
			message(STATUS "ARM NONE EABI Toolchain - found tool (${tool})")
		endif()

		if (${tool_error})
			message(FATAL_ERROR "ARM NONE EABI Toolchain - Tool validation failed. One or more tool path was missing.")
		endif()
	endforeach()

	message(STATUS "ARM NONE EABI Toolchain - root validated!")
endmacro()

macro(FIND_ARM_NONE_EABI_ROOT toolchain_root)
	find_program(ARM_NONE_EABI_GCC "arm-none-eabi-gcc")
	if (ARM_NONE_EABI_GCC)
		message(STATUS "ARM NONE EABI Toolchain - Compiler already on path (${ARM_NONE_EABI_GCC}).")
		cmake_path(GET ARM_NONE_EABI_GCC PARENT_PATH ARM_NONE_EABI_PATH)
		cmake_path(GET ARM_NONE_EABI_PATH PARENT_PATH ARM_NONE_EABI_PATH)
		message(STATUS "ARM NONE EABI Toolchain - Recovered root path (${ARM_NONE_EABI_PATH}).")
		
		validate_arm_none_eabi_root(${ARM_NONE_EABI_PATH})

		set(${toolchain_root} ${ARM_NONE_EABI_PATH})
	elseif (DEFINED ENV{ARM_NONE_EABI_ROOT})
		message(STATUS "ARM NONE EABI Toolchain - Arm toolchain specificed in environment variable.")

		set(ARM_NONE_EABI_ROOT ENV{ARM_NONE_EABI_ROOT})
		validate_arm_none_eabi_root(${ARM_NONE_EABI_ROOT})
		set(${toolchain_root} ${ARM_NONE_EABI_ROOT})
	else()
		if(DEFINED ENV{IN_NIX_SHELL})
			message(FATAL_ERROR "ARM NONE EABI Toolchain - In a nix shell. Expected tools on the path. Are you in the right environment?")
		else()
			message(FATAL_ERROR "ARM NONE EABI Toolchain - Unable to locate arm none eabi tools.")
		endif()
	endif()
endmacro() 

set(CMAKE_CXX_STANDARD 17 CACHE STRING "")
set(CMAKE_CXX_STANDARD_REQUIRED ON CACHE BOOL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "${CMAKE_BINARY_DIR}/bin" CACHE PATH "")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  "${CMAKE_BINARY_DIR}/lib" CACHE PATH "")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  "${CMAKE_BINARY_DIR}/lib" CACHE PATH "")

find_arm_none_eabi_root(STM32_TOOLCHAIN_PATH)

if(NOT STM32_TOOLCHAIN_PATH)
	message(FATAL_ERROR "stm32 toolchain path not set. Use find_arm_none_eabi_root macro")
else()
	file(TO_CMAKE_PATH "${STM32_TOOLCHAIN_PATH}" STM32_TOOLCHAIN_PATH)
endif()

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(STM32_TARGET_TRIPLET "arm-none-eabi")

set(TOOLCHAIN_SYSROOT  "${STM32_TOOLCHAIN_PATH}/${STM32_TARGET_TRIPLET}")
set(TOOLCHAIN_BIN_PATH "${STM32_TOOLCHAIN_PATH}/bin")
set(TOOLCHAIN_INC_PATH "${STM32_TOOLCHAIN_PATH}/${STM32_TARGET_TRIPLET}/include")
set(TOOLCHAIN_LIB_PATH "${STM32_TOOLCHAIN_PATH}/${STM32_TARGET_TRIPLET}/lib")

find_program(CMAKE_OBJCOPY NAMES ${STM32_TARGET_TRIPLET}-objcopy PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_OBJDUMP NAMES ${STM32_TARGET_TRIPLET}-objdump PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_SIZE NAMES ${STM32_TARGET_TRIPLET}-size PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_DEBUGGER NAMES ${STM32_TARGET_TRIPLET}-gdb PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CPPFILT NAMES ${STM32_TARGET_TRIPLET}-c++filt PATHS ${TOOLCHAIN_BIN_PATH})

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
find_program(CMAKE_C_COMPILER NAMES ${STM32_TARGET_TRIPLET}-gcc PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CXX_COMPILER NAMES ${STM32_TARGET_TRIPLET}-g++ PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_ASM_COMPILER NAMES ${STM32_TARGET_TRIPLET}-gcc PATHS ${TOOLCHAIN_BIN_PATH})
