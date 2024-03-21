include_directories(${CMAKE_SOURCE_DIR}/external/eigen/) 

set(SRCS 
calibrate.cpp
${CMAKE_SOURCE_DIR}/arc/src/sensors/MPU6050.cpp
)

set(HEADERS 
${CMAKE_SOURCE_DIR}/arc/include/arc/common/common.hpp
${CMAKE_SOURCE_DIR}/arc/include/arc/common/constants.hpp
${CMAKE_SOURCE_DIR}/arc/include/arc/common/i2c.hpp
${CMAKE_SOURCE_DIR}/arc/include/arc/common/log.hpp
${CMAKE_SOURCE_DIR}/arc/include/arc/common/stopwatch.hpp
${CMAKE_SOURCE_DIR}/arc/include/arc/control/kalman.hpp
${CMAKE_SOURCE_DIR}/arc/include/arc/sensors/MPU6050.hpp
)

add_executable(imu_calibrate ${SRCS} ${HEADERS})
target_include_directories(imu_calibrate PRIVATE ${CMAKE_SOURCE_DIR}/arc/include/arc)

# pull in common dependencies
target_link_libraries(imu_calibrate pico_stdlib hardware_i2c hardware_xosc)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(imu_calibrate)

# Use the fast/compact SDK/bootrom implementation of operations
pico_set_float_implementation(imu_calibrate pico)
pico_set_double_implementation(imu_calibrate pico)