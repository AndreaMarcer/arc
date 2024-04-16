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

add_executable(record record.cpp ${SRCS} ${HEADERS})
target_include_directories(record PRIVATE ${CMAKE_SOURCE_DIR}/arc/include/arc)
target_link_libraries(record pico_stdlib hardware_i2c hardware_xosc)
pico_add_extra_outputs(record)
pico_set_float_implementation(record pico)
pico_set_double_implementation(record pico)