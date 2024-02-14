cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(modbus-rp2040 C CXX ASM)

add_library(modbus-rp2040
    INTERFACE
)

set(CMAKE_C_STANDART 11)
set(CMAKE_CXX_STANDART 17)

target_include_directories(modbus-rp2040
    INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_sources(modbus-rp2040
    INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/modbus.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/modbus_data.cpp
)

target_link_libraries(modbus-rp2040
    INTERFACE
    pico_stdlib
)