#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"
#include <math.h>
#include <stdint.h>
#include "line_follower.h"
#define ADC_SLEEP_MS 10

volatile bool is_black_ready = false;  
volatile bool is_black = false;  

int main() {
    stdio_init_all(); 
    sleep_ms(10000);
    printf("Start\n");
    setup_line_follower();

    struct repeating_timer sampling_timer;
    bool success = add_repeating_timer_ms(ADC_SLEEP_MS, polling_function, NULL, &sampling_timer);
    while(1) {
        if (is_black_ready) {              // Check if new ADC data is ready
            is_black_ready = false;        // Reset the flag
            printf("isBlack: %u\n", is_black);  // Print the shared ADC reading
        }
    }
    printf("END\n");
    return 0;
}

