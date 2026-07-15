cmake_minimum_required(VERSION 3.24)

project(software_v0_2 C ASM)

set(T2G_FLASH_COMMAND "" CACHE STRING
    "Complete programmer command. It must accept cm7_0.hex and cm7_1.hex as its final two arguments.")

set(T2G_COMMON_DEFINES
    tviibh4m
    CYT4BB7CEE
    CY_USE_PSVP=0
    CY_MCU_rev_b
    CPU_BOARD_REVB)

set(T2G_INCLUDE_DIRS
    libraries/sdk/common/hdr
    libraries/sdk/common/hdr/cmsis/include
    libraries/sdk/common/src/drivers
    libraries/sdk/common/src/mw
    libraries/sdk/tviibh4m/hdr
    libraries/sdk/tviibh4m/hdr/ip
    libraries/sdk/tviibh4m/hdr/mcureg
    libraries/sdk/tviibh4m/src
    libraries/sdk/tviibh4m/src/interrupts
    libraries/sdk/tviibh4m/src/mw
    libraries/sdk/tviibh4m/src/drivers
    libraries/sdk/tviibh4m/src/system
    libraries/sdk/common/hdr/cmsis/dsp
    libraries/zf_common
    libraries/zf_driver
    libraries/zf_device
    libraries/zf_components
    project/user
    project/code)

function(t2g_iar_sources core output)
    set(project_file "${CMAKE_CURRENT_SOURCE_DIR}/project/iar/project_config/cyt4bb7_cm_7_${core}.ewp")
    file(STRINGS "${project_file}" source_lines REGEX "<name>.*\\.(c|s|S)</name>")

    set(sources)
    foreach(line IN LISTS source_lines)
        string(REGEX REPLACE ".*<name>([^<]+)</name>.*" "\\1" source "${line}")
        string(REPLACE "\\" "/" source "${source}")
        string(REPLACE "$PROJ_DIR$/../../.." "${CMAKE_CURRENT_SOURCE_DIR}" source "${source}")
        string(REPLACE "$PROJ_DIR$/../.." "${CMAKE_CURRENT_SOURCE_DIR}/project" source "${source}")

        if(source MATCHES "/iar/cy_syslib_iar\\.s$")
            set(source "${CMAKE_CURRENT_SOURCE_DIR}/libraries/sdk/common/src/drivers/syslib/gcc/cy_syslib_gcc.S")
        elseif(source MATCHES "/startup/iar/startup_cm7\\.s$")
            set(source "${CMAKE_CURRENT_SOURCE_DIR}/libraries/sdk/common/src/startup/gcc/startup_cm7.S")
        endif()
        list(APPEND sources "${source}")
    endforeach()

    list(REMOVE_DUPLICATES sources)

    foreach(source IN LISTS sources)
        if(NOT EXISTS "${source}")
            message(FATAL_ERROR "IAR source listed by ${project_file} is missing: ${source}")
        endif()
    endforeach()
    set(${output} "${sources}" PARENT_SCOPE)
endfunction()

function(t2g_firmware core)
    t2g_iar_sources(${core} sources)
    set(target cm7_${core})
    add_executable(${target} ${sources})

    target_include_directories(${target} PRIVATE ${T2G_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE
        m
        ${CMAKE_CURRENT_SOURCE_DIR}/libraries/zf_device/zf_device_config.a)
    target_compile_definitions(${target} PRIVATE ${T2G_COMMON_DEFINES} CY_CORE_CM7_${core})
    target_compile_options(${target} PRIVATE
        -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16
        -ffunction-sections -fdata-sections -fno-common
        $<$<COMPILE_LANGUAGE:C>:-Og -g3 -Wall -Wextra -Wno-error=incompatible-pointer-types>)
    target_link_options(${target} PRIVATE
        -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16
        -nostartfiles -Wl,--gc-sections -Wl,--cref
        -Wl,-L,${CMAKE_CURRENT_SOURCE_DIR}/cmake/linker
        -Wl,-T,${CMAKE_CURRENT_SOURCE_DIR}/cmake/linker/cm7_${core}.ld
        -Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}/${target}.map
        --specs=nano.specs --specs=nosys.specs)

    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> ${CMAKE_CURRENT_BINARY_DIR}/${target}.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>
        BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/${target}.hex
        COMMENT "Generating ${target}.hex")
endfunction()

t2g_firmware(0)
t2g_firmware(1)
add_custom_target(firmware ALL DEPENDS cm7_0 cm7_1)

if(T2G_FLASH_COMMAND)
    separate_arguments(T2G_FLASH_COMMAND NATIVE_COMMAND "${T2G_FLASH_COMMAND}")
    add_custom_target(flash DEPENDS firmware
        COMMAND ${T2G_FLASH_COMMAND}
            ${CMAKE_CURRENT_BINARY_DIR}/cm7_0.hex
            ${CMAKE_CURRENT_BINARY_DIR}/cm7_1.hex
        USES_TERMINAL
        COMMENT "Programming both CM7 images")
else()
    add_custom_target(flash
        COMMAND ${CMAKE_COMMAND} -E echo
            "Set T2G_FLASH_COMMAND to the programmer command before using the flash target."
        COMMAND ${CMAKE_COMMAND} -E false)
endif()
