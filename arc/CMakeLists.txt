
include_directories(${CMAKE_SOURCE_DIR}/external/eigen/) 

set(SRCS 
src/main.cpp
src/sensors/MPU6050.cpp
)

set(HEADERS 
include/arc/common/common.hpp
include/arc/common/constants.hpp
include/arc/common/i2c.hpp
include/arc/common/log.hpp
include/arc/common/stopwatch.hpp
include/arc/control/kalman.hpp
include/arc/sensors/MPU6050.hpp
)

add_executable(arc ${SRCS} ${HEADERS})
target_include_directories(arc PRIVATE include/arc)

# pull in common dependencies
target_link_libraries(arc pico_stdlib hardware_i2c hardware_xosc)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(arc)

# Use the fast/compact SDK/bootrom implementation of operations
pico_set_float_implementation(arc pico)
pico_set_double_implementation(arc pico)