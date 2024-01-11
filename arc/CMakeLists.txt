set(SRCS 
    src/main.c
)

add_executable(arc ${SRCS})

# pull in common dependencies
target_link_libraries(arc pico_stdlib hardware_i2c)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(arc)
