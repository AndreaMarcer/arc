include_directories(${CMAKE_SOURCE_DIR}/external/eigen/) 

set(SRCS 
${CMAKE_SOURCE_DIR}/arc/src/sensors/MPU6050.cpp
)

set(HEADERS 
${CMAKE_SOURCE_DIR}/arc/include/arc/common/common.hpp
${CMAKE_SOURCE_DIR}/arc/include/arc/common/constants.hpp
${CMAKE_SOURCE_DIR}/arc/include/arc/common/i2c.hpp
${CMAKE_SOURCE_DIR}/arc/include/arc/common/log.hpp
${CMAKE_SOURCE_DIR}/arc/include/arc/sensors/MPU6050.hpp
)

add_executable(calibrate calibrate.cpp ${SRCS} ${HEADERS})
target_include_directories(calibrate PRIVATE ${CMAKE_SOURCE_DIR}/arc/include/arc)
target_link_libraries(calibrate pico_stdlib hardware_i2c)
pico_add_extra_outputs(calibrate)
pico_set_float_implementation(calibrate pico)
pico_set_double_implementation(calibrate pico)

add_executable(scale_acc scale_acc.cpp ${SRCS} ${HEADERS})
target_include_directories(scale_acc PRIVATE ${CMAKE_SOURCE_DIR}/arc/include/arc)
target_link_libraries(scale_acc pico_stdlib hardware_i2c)
pico_add_extra_outputs(scale_acc)
pico_set_float_implementation(scale_acc pico)
pico_set_double_implementation(scale_acc pico)

add_executable(scale_gyro scale_gyro.cpp ${SRCS} ${HEADERS})
target_include_directories(scale_gyro PRIVATE ${CMAKE_SOURCE_DIR}/arc/include/arc)
target_link_libraries(scale_gyro pico_stdlib hardware_i2c)
pico_add_extra_outputs(scale_gyro)
pico_set_float_implementation(scale_gyro pico)
pico_set_double_implementation(scale_gyro pico)