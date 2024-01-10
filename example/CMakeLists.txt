cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(modbus-example)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# include modbus-rp2040 library
include(${CMAKE_CURRENT_LIST_DIR}/../CMakeLists.txt)

add_executable(modbus-example
        main.cpp
)

# pull in common dependencies
target_link_libraries(modbus-example
    modbus-rp2040
    pico_stdlib
    pico_multicore
)

# create map/bin/hex file etc.
pico_add_extra_outputs(modbus-example)

pico_enable_stdio_usb(  modbus-example 0)
pico_enable_stdio_uart( modbus-example 1)