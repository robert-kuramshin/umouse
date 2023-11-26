cmake_minimum_required(VERSION 3.25)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
# Pull in SDK by downloading the pico_sdk_import

include(pico_sdk_import.cmake)

project(umouse C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall)
pico_sdk_init()

# Define flag for Pico W
if(${PICO_BOARD} STREQUAL pico_w)
    add_compile_definitions(PICO_W_BOARD)
    message("Compiling examples for Pico W")
else()
    message("Compiling examples for Pico")
endif()

add_subdirectory(example)