add_executable(buddy2 buddy2.c)
# pull in common dependencies
target_link_libraries(buddy2 pico_stdlib hardware_pwm wheel_encoder_driver ultrasonic_driver)

# # enable usb output
pico_enable_stdio_usb(buddy2 1)
pico_enable_stdio_uart(buddy2 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(buddy2)

# add url via pico_set_program_url
example_auto_set_url(buddy2)

# add_executable(WifiGY511
#         WifiGY511.c
#         )
# target_compile_definitions(WifiGY511 PRIVATE
#         WIFI_SSID=\"$ENV{WIFI_SSID}\"
#         WIFI_PASSWORD=\"$ENV{WIFI_PASSWORD}\"
#         NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
#         LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
#         PING_USE_SOCKETS=1
#         )
# target_include_directories(WifiGY511 PRIVATE
#         ${CMAKE_CURRENT_LIST_DIR}
#         ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
#         ${PICO_LWIP_CONTRIB_PATH}/apps/ping
#         )
# target_link_libraries(WifiGY511
#         pico_cyw43_arch_lwip_sys_freertos
#         pico_stdlib
#         pico_lwip_iperf
#         FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
#         pico_stdlib hardware_i2c
#         )

    
# # enable usb output
# pico_enable_stdio_usb(WifiGY511 1)
# pico_add_extra_outputs(WifiGY511)

add_executable(WifiMotor 
        WifiMotor.c
        )

target_compile_definitions(WifiMotor PRIVATE
        WIFI_SSID=\"$ENV{WIFI_SSID}\"
        WIFI_PASSWORD=\"$ENV{WIFI_PASSWORD}\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        PING_USE_SOCKETS=1
        )
target_include_directories(WifiMotor PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
        ${PICO_LWIP_CONTRIB_PATH}/apps/ping
        )
# pull in common dependencies
target_link_libraries(WifiMotor 
        pico_stdlib 
        hardware_pwm 
        wheel_encoder_driver 
        ultrasonic_driver
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )

# # enable usb output
pico_enable_stdio_usb(WifiMotor 1)
pico_enable_stdio_uart(WifiMotor 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(WifiMotor)

add_executable(PicoServerMotor
        PicoServerMotor.c
        )
target_compile_definitions(PicoServerMotor PRIVATE
        WIFI_SSID=\"$ENV{WIFI_SSID}\"
        WIFI_PASSWORD=\"$ENV{WIFI_PASSWORD}\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        PING_USE_SOCKETS=1
        )
target_include_directories(PicoServerMotor PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
        ${PICO_LWIP_CONTRIB_PATH}/apps/ping
        )
target_link_libraries(PicoServerMotor
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )
    
    
# enable usb output
pico_enable_stdio_usb(PicoServerMotor 1)
pico_add_extra_outputs(PicoServerMotor)
