add_library(ultrasonic_driver ultrasonic_driver.h ultrasonic_driver.c)

target_link_libraries(ultrasonic_driver pico_stdlib hardware_gpio hardware_timer)

target_include_directories(ultrasonic_driver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")