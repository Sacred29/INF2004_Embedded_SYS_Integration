add_executable(RemoteControl
RemoteControl.c
        )
target_compile_definitions(RemoteControl PRIVATE
        WIFI_SSID=\"$ENV{WIFI_SSID}\"
        WIFI_PASSWORD=\"$ENV{WIFI_PASSWORD}\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        PING_USE_SOCKETS=1
        )
target_include_directories(RemoteControl PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
        ${PICO_LWIP_CONTRIB_PATH}/apps/ping
        )
target_link_libraries(RemoteControl
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        pico_stdlib hardware_i2c
        )
    
    
# enable usb output
pico_enable_stdio_usb(RemoteControl 1)
pico_add_extra_outputs(RemoteControl)

add_executable(Car 
        Car.c
        )

target_compile_definitions(Car PRIVATE
        WIFI_SSID=\"$ENV{WIFI_SSID}\"
        WIFI_PASSWORD=\"$ENV{WIFI_PASSWORD}\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        PING_USE_SOCKETS=1
        )
target_include_directories(Car PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
        ${PICO_LWIP_CONTRIB_PATH}/apps/ping
        )
# pull in common dependencies
target_link_libraries(Car 
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
pico_enable_stdio_usb(Car 1)
pico_enable_stdio_uart(Car 0)