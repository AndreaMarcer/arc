
set(SRCS 
src/main.cpp
src/sensors/MPU6050.cpp
)

set(HEADERS 
include/arc/sensors/MPU6050.hpp
include/arc/common/constants.hpp
include/arc/common/common.hpp
include/arc/common/log.hpp
include/arc/math/matrix.hpp
)

add_executable(arc ${SRCS} ${HEADERS})
target_include_directories(arc PRIVATE include/arc)

# pull in common dependencies
target_link_libraries(arc pico_stdlib hardware_i2c)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(arc)

# Use the fast/compact SDK/bootrom implementation of operations
pico_set_float_implementation(arc pico)
pico_set_double_implementation(arc pico)