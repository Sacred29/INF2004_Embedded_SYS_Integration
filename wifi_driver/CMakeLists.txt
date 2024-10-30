if (EXISTS ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c)
    add_executable(WifiDriver
            WifiDriver.c
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
            )
    target_compile_definitions(WifiDriver PRIVATE
            WIFI_SSID=\"$ENV{WIFI_SSID}\"
            WIFI_PASSWORD=\"$ENV{WIFI_PASSWORD}\"
            NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
            LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
            PING_USE_SOCKETS=1
            )
    target_include_directories(WifiDriver PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
            ${PICO_LWIP_CONTRIB_PATH}/apps/ping
            )
    target_link_libraries(WifiDriver
            pico_cyw43_arch_lwip_sys_freertos
            pico_stdlib
            pico_lwip_iperf
            FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
            )
    
    
# enable usb output
pico_enable_stdio_usb(WifiDriver 1)
pico_add_extra_outputs(WifiDriver)

endif()