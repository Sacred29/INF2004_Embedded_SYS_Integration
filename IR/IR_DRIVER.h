// IR_DRIVER.h
#ifndef IR_DRIVER_H
#define IR_DRIVER_H

#include <stdint.h>
#include "pico/stdlib.h"

typedef struct {
    uint8_t gpio_pin;            // GPIO pin for the IR sensor
    uint8_t adc_channel;         // ADC channel for the sensor
    uint32_t interval_ms;        // Timer interval in milliseconds
    uint32_t last_readings[3];   // Buffer to store the last 3 readings
    uint64_t sum;                // Sum of values in the buffer
    uint8_t count;               // Count of readings taken (up to 3)
    uint8_t index;               // Index for the circular buffer
} Sensor;

#define NUM_SENSORS 2     // Number of sensors
extern Sensor sensors[NUM_SENSORS]; // Declare an array of Sensor structs

void setup_ir_sensors();
void setup_timers();
bool sampling_timer_function(__unused struct repeating_timer *t, int sensor_index);

#endif // IR_DRIVER_H
