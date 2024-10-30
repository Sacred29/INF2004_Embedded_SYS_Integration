# CMakeLists.txt
add_library(line_follower line_follower.c)

# Specify include directory for the library
target_include_directories(line_follower PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Link any dependencies (e.g., pico_stdlib for stdio and FreeRTOS if required)
target_link_libraries(line_follower pico_stdlib hardware_adc hardware_timer)
