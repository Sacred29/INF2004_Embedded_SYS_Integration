add_executable(station2A station2A.c)
# pull in common dependencies
target_link_libraries(station2A pico_stdlib hardware_pwm wheel_encoder_driver line_follower barcode_decoder)

# # enable usb output
pico_enable_stdio_usb(station2A 1)
pico_enable_stdio_uart(station2A 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(station2A)



