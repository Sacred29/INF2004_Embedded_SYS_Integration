// IR_DRIVER.c
#include "IR_DRIVER.h"

void setup_ir_sensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        gpio_disable_pulls(sensors[i].gpio_pin);           // Disable pull-up/down resistors
        gpio_set_input_enabled(sensors[i].gpio_pin, false); // Disable digital input
        adc_select_input(sensors[i].adc_channel);           // Set ADC channel
    }
}

void setup_timers() {
    static struct repeating_timer timers[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
        add_repeating_timer_ms(
            sensors[i].interval_ms, // Use the interval from the struct for each timer
            (repeating_timer_callback_t)sampling_timer_function,
            (void *)(uintptr_t)i,  // Pass the sensor index as an argument (typecast to void pointer)
            &timers[i]
        );
    }
}

bool sampling_timer_function(__unused struct repeating_timer *t, int sensor_index) {
    if (sensor_index < NUM_SENSORS) {
        // Select the ADC channel for the current sensor
        adc_select_input(sensors[sensor_index].adc_channel);

        // Read ADC value
        uint32_t reading = adc_read();

        // Update the moving average calculation
        uint8_t idx = sensors[sensor_index].index; // Current index for circular buffer

        // Subtract the oldest value from the sum
        sensors[sensor_index].sum -= sensors[sensor_index].last_readings[idx];
        
        // Replace the oldest value with the new reading
        sensors[sensor_index].last_readings[idx] = reading;
        
        // Add the new reading to the sum
        sensors[sensor_index].sum += reading;

        // Update the index in a circular manner
        sensors[sensor_index].index = (idx + 1) % 3;

        // Increment count up to a maximum of 3 (for the window size)
        if (sensors[sensor_index].count < 3) {
            sensors[sensor_index].count++;
        }
    }
    return true; // Keep the timer active
}
