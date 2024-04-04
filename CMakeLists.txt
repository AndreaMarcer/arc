cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(external/pico-sdk/external/pico_sdk_import.cmake)

#include(pico_extras_import_optional.cmake)

project(ARC C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_compile_options(
  -Wall
  -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
  -Wno-unused-function # we have some for the docs that aren't called
)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

# Logging defines
add_compile_definitions(LOG_DEBUG LOG_LEVEL=3 LOG_COLOR)

# Include project
add_subdirectory(arc)
add_subdirectory(tools/imu/calibrate)
