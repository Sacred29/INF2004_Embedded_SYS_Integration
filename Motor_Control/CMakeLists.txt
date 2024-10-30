# CMakeLists.txt
add_library(motor_control motor_control.c)

# Specify include directory for the library
target_include_directories(motor_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Link any dependencies (e.g., pico_stdlib for stdio and FreeRTOS if required)
target_link_libraries(motor_control pico_stdlib hardware_pwm hardware_timer)