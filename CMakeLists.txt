# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(macro_itm C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(macro_itm macro_itm.c )

pico_set_program_name(macro_itm "macro_itm")
pico_set_program_version(macro_itm "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(macro_itm 0)
pico_enable_stdio_usb(macro_itm 1)

# Add the standard library to the build
target_link_libraries(macro_itm
    pico_stdlib
    pico_btstack_ble
    pico_btstack_classic
    pico_btstack_cyw43
    pico_cyw43_arch_none
)

# Add the standard include files to the build
target_include_directories(macro_itm PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_btstack_make_gatt_header(macro_itm PRIVATE "${CMAKE_CURRENT_LIST_DIR}/macro_itm.gatt")

pico_add_extra_outputs(macro_itm)

