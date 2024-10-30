add_executable(Buddy4
        buddy4.c
        )

pico_enable_stdio_usb(Buddy4 1)
pico_enable_stdio_uart(Buddy4 0)

target_link_libraries(Buddy4 pico_stdlib hardware_i2c)
pico_add_extra_outputs(Buddy4)