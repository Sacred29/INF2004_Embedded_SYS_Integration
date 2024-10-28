// main.c
#include "IR_DRIVER.h"
#include <stdio.h>

// Initialize sensors with GPIO pins, ADC channels, time intervals, and default values for moving average
Sensor sensors[NUM_SENSORS] = {
    {26, 0, 1000, {0, 0, 0}, 0, 0, 0}, // Sensor 1: GPIO 26, ADC channel 0, interval 100ms
    {27, 1, 5000, {0, 0, 0}, 0, 0, 0}  // Sensor 2: GPIO 28, ADC channel 1, interval 200ms
};

int main() {
    setup_ir_sensors(); // Configure the sensors
    setup_timers();     // Set up timers for each sensor

    // Main loop to print sensor data periodically
    while (1) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            // Calculate the moving average for the current sensor
            uint32_t average = sensors[i].sum / (sensors[i].count > 0 ? sensors[i].count : 1); // Avoid division by zero

            // Print the sensor data
            printf("Sensor %d (GPIO %d, ADC %d, Interval %d ms) Average: %u\n",
                   i, sensors[i].gpio_pin,
                   sensors[i].adc_channel, sensors[i].interval_ms,
                   average);
        }
        sleep_ms(1000); // Print every 1 second to reduce output frequency
    }
    return 0;
}
