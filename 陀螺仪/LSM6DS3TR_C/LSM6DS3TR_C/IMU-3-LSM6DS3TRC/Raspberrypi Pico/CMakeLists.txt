cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(lsm6ds3trc C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(lsm6ds3trc main.c lsm6ds3trc.c)

pico_add_extra_outputs(lsm6ds3trc)

#Pull in our pico_stdlib which aggregates commonly used features
target_link_libraries(lsm6ds3trc pico_stdlib hardware_i2c hardware_spi)

# enable usb output, disable uart output
pico_enable_stdio_usb(lsm6ds3trc 1)
pico_enable_stdio_uart(lsm6ds3trc 0)

