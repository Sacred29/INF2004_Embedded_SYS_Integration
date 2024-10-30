# CMakeLists.txt
add_library(barcode_decoder barcode_decoder.c)

# Specify include directory for the library
target_include_directories(barcode_decoder PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Link any dependencies (e.g., pico_stdlib for stdio and FreeRTOS if required)
target_link_libraries(barcode_decoder pico_stdlib hardware_adc hardware_timer)
