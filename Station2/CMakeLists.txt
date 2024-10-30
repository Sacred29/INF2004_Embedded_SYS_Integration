
if (EXISTS ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c)
add_executable(Station2
station2.c
${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
)

target_compile_definitions(Station2 PRIVATE
WIFI_SSID=\"${WIFI_SSID}\"
WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
NO_SYS=0           # don't want NO_SYS (generally this would be in your lwipopts.h)
LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
PING_USE_SOCKETS=1
)
target_include_directories(Station2 PRIVATE
${CMAKE_CURRENT_LIST_DIR}
${PICO_LWIP_CONTRIB_PATH}/apps/ping
)


pico_enable_stdio_usb(Station2 1)


target_link_libraries(Station2
motor_control
line_follower
wheel_encoder_driver 
hardware_adc
pico_cyw43_arch_lwip_sys_freertos
pico_stdlib
pico_lwip_iperf
FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)
pico_add_extra_outputs(Station2)
endif()
