cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# initialize pico-sdk from submodule note: this must happen before project()
# include(pico-sdk/pico_sdk_init.cmake)
include(pico-sdk/external/pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(robot src/main.c)

pico_generate_pio_header(robot
                         ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio)

target_sources(robot PRIVATE src/main.c src/encoder.c src/motor_controller.c
                             src/optical_sensor.c)

# pull in common dependencies
target_link_libraries(robot pico_stdlib hardware_pio hardware_pwm hardware_adc)

# enable usb output, disable uart output
pico_enable_stdio_usb(robot 1)
pico_enable_stdio_uart(robot 0)

pico_add_extra_outputs(robot)

# create map/bin/hex file etc.
pico_add_extra_outputs(robot)
