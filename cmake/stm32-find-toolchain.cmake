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

	set(path_error FALSE)
	foreach(toolchain_path ${path_list})
		if (NOT EXISTS ${toolchain_path})
			message(WARNING "ARM NONE EABI Toolchain - path does not exist (${toolchain_path})")
			set(path_error TRUE)
		else()
			message(STATUS "ARM NONE EABI Toolchain - found path (${toolchain_path})")
		endif()

	endforeach()

	if (${path_error})
		message(FATAL_ERROR "ARM NONE EABI Toolchain - Tool validation failed. One or more tool path was missing.")
	endif()

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
