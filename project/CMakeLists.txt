add_executable(buddy2 buddy2.c)
# pull in common dependencies
target_link_libraries(buddy2 pico_stdlib hardware_pwm)

# # enable usb output
pico_enable_stdio_usb(buddy2 1)
pico_enable_stdio_uart(buddy2 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(buddy2)

# add url via pico_set_program_url
example_auto_set_url(buddy2)

