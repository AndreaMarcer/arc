add_executable(arc
)

# pull in common dependencies
target_link_libraries(arc pico_stdlib)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(arc)
