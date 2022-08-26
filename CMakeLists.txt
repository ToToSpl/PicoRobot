cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(robot
        src/main.c
        )

# pull in common dependencies
target_link_libraries(robot pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(robot)