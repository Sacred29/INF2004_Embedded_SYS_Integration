// line_follower.c
#include "barcode_decoder.h"      // Include the header file for function declarations
#include <stdio.h>
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <math.h>

void setup_barcode_decoder() {
    adc_init();
    gpio_disable_pulls(ADC_GPIO);            // Disable GPIO pull-ups/downs
    gpio_set_input_enabled(ADC_GPIO, false); // Disable digital input for continuous ADC reading
    adc_select_input(ADC_Input);
}


// Polling function for repeated ADC sampling
bool polling_function(__unused struct repeating_timer *t) {
    uint32_t reading = adc_read();

    static uint32_t data[NUM_SLOT] = {0};
    static uint32_t index = 0;
    static uint64_t sum = 0;
    static uint32_t count = 0;

    static uint32_t adc_reading;

    if (count < NUM_SLOT) count++; // Increment count till it reaches NUM_SLOT

    // Moving Average Calculation
    sum -= data[index];         // Subtract the oldest element from sum
    data[index] = reading;      // Add new element to the data array
    sum += data[index];         // Add the new element to sum
    index = (index + 1) % NUM_SLOT;  // Update index for circular buffer

    adc_reading = (uint32_t)ceil((double)sum / count);

    static uint32_t min_threshold = 4095;
    static uint32_t max_threshold = 0;
    static uint32_t contrast_threshold = 4095;

    if (adc_reading < min_threshold) {
        min_threshold = adc_reading;
        contrast_threshold = (uint32_t)ceil((double)(min_threshold+max_threshold) / 2);
    }
    if (adc_reading > max_threshold) {
        max_threshold = adc_reading;
        contrast_threshold = (uint32_t)ceil((double)(min_threshold+max_threshold) / 2);
    }
    
    return true;
}
