# https://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
macro(subdirlist result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

macro(ateam_add_targets root_dir linker_file device_prefix)
    set(binaries_dir ${root_dir}/bin)
    set(common_dir ${root_dir}/common)

    # get common source files
    file(GLOB_RECURSE LOCAL_COMMON_ASM_FILES "${common_dir}/*.s")
	file(GLOB_RECURSE LOCAL_COMMON_C_FILES "${common_dir}/*.c")
	file(GLOB_RECURSE LOCAL_COMMON_CXX_FILES "${common_dir}/*.cpp")

    # get common header files
    header_directories(LOCAL_COMMON_INCLUDE_DIRS ${common_dir})
    set(COMMON_PACKETS_DIR "${PROJECT_SOURCE_DIR}/software-communication/")

    subdirlist(bin_folders "${binaries_dir}")
    foreach(bin_dir ${bin_folders})
        message(INFO " Adding target: ${bin_dir}")
        set(target_name ${bin_dir})
        set(bin_dir ${root_dir}/bin/${bin_dir})
        message(INFO "${bin_dir}")

        file(GLOB_RECURSE BIN_ASM_FILES "${bin_dir}/*.s")
        file(GLOB_RECURSE BIN_C_FILES "${bin_dir}/*.c")
        file(GLOB_RECURSE BIN_CXX_FILES "${bin_dir}/*.cpp")
    
        add_executable(${target_name}.elf
            ${BIN_ASM_FILES}
            ${BIN_C_FILES}
            ${BIN_CXX_FILES}
            ${LOCAL_COMMON_ASM_FILES}
            ${LOCAL_COMMON_C_FILES}
            ${LOCAL_COMMON_CXX_FILES}
        )
    
        header_directories(BIN_INCLUDE_DIRS ${bin_dir})
        message(INFO "${BIN_INCLUDE_DIRS}")
        target_include_directories(${target_name}.elf PRIVATE
            ${BIN_INCLUDE_DIRS}
            ${LOCAL_COMMON_INCLUDE_DIRS}
            ${COMMON_PACKETS_DIR}
        )
    
        target_compile_definitions(${target_name}.elf PRIVATE
            ${${device_prefix}_DEFINITIONS}
        )
    
        target_compile_options(${target_name}.elf PRIVATE
            $<$<COMPILE_LANGUAGE:C>:${${device_prefix}_C_OPTIONS}>
            $<$<COMPILE_LANGUAGE:CXX>:${${device_prefix}_CXX_OPTIONS}>
            -Wdouble-promotion
            -Werror=double-promotion
        ) 
    
        target_link_options(${target_name}.elf PRIVATE ${${device_prefix}_LINKER_OPTIONS} -T ${linker_file})
    
        add_custom_target(${target_name}
            arm-none-eabi-objcopy -Obinary "${target_name}.elf" "${target_name}.bin"
            WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
            DEPENDS ${target_name}.elf
            COMMENT "create flat binary for programming"
        )
    
        add_custom_target(${target_name}-prog
            openocd -f "${${device_prefix}_OPENOCD_CFG}" -c "program bin/${target_name}.bin verify reset exit ${STM32_FLASH_START_ADDR}"
        )

        add_custom_target(${target_name}-gdb
         	${CMAKE_CURRENT_SOURCE_DIR}/util/attach_gdb.sh ${${device_prefix}_OPENOCD_CFG} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_name}.elf
        )
    endforeach()
endmacro()