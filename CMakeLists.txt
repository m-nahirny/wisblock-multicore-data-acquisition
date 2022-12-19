cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(lis3dh_i2c C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(lis3dh_i2c
        lis3dh_i2c.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(lis3dh_i2c pico_stdlib hardware_i2c)

# enable uart output, disable usb output
pico_enable_stdio_uart(lis3dh_i2c 1)
pico_enable_stdio_usb(lis3dh_i2c 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(lis3dh_i2c)

# add url via pico_set_program_url
#example_auto_set_url(lis3dh_i2c)

# pull in common dependencies
target_link_libraries(lis3dh_i2c pico_stdlib pico_multicore hardware_adc)