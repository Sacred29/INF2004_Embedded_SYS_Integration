#include <stdio.h>
#include <time.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

float wheel_diameter = 6.5;       // Wheel diameter in centimeters
float pulses_per_revolution = 20; // Adjust based on the number of marks or holes the IR sensor detects per wheel revolution// Calculate the wheel speed in meters per second (m/s)

uint32_t left_pulse_count = 0;
uint32_t right_pulse_count = 0;
uint32_t left_prev_pulse_count = 0;
uint32_t right_prev_pulse_count = 0;

void setupWheelEncoderPins(uint left_wheel_pin, uint right_wheel_pin)
{
     gpio_init(left_wheel_pin);
     gpio_set_dir(left_wheel_pin, GPIO_IN);
     gpio_pull_up(left_wheel_pin);

     gpio_init(right_wheel_pin);
     gpio_set_dir(right_wheel_pin, GPIO_IN);
     gpio_pull_up(right_wheel_pin);
}

void leftWheelPulseCounting()
{
     left_pulse_count++;
}

void rightWheelPulseCounting()
{
     right_prev_pulse_count++;
}

float getLeftWheelSpeed(float pulse_width)
{
     uint32_t left_current_pulse_count = left_pulse_count - left_prev_pulse_count;

     float microsec_per_pulse = pulse_width / left_current_pulse_count;

     // Time between pulses (in seconds)
     float sec_per_pulse = microsec_per_pulse * 0.000001f; // Convert microseconds to seconds

     // Wheel circumference
     float wheel_circumference = 3.14159 * wheel_diameter;

     // The number of complete revolution per second
     float revolutions_per_second = 1 / (sec_per_pulse * pulses_per_revolution);

     // Speed in cemtimeters per second (cm/s)
     float speed_meters_per_sec = revolutions_per_second * wheel_circumference;

     left_prev_pulse_count = left_pulse_count;

     return speed_meters_per_sec;
}

float getRightWheelSpeed(float pulse_width)
{
     uint32_t right_current_pulse_count = right_pulse_count - right_prev_pulse_count;

     float microsec_per_pulse = pulse_width / right_current_pulse_count;

     // Time between pulses (in seconds)
     float sec_per_pulse = microsec_per_pulse * 0.000001f; // Convert microseconds to seconds

     // Wheel circumference
     float wheel_circumference = 3.14159 * wheel_diameter;

     // The number of complete revolution per second
     float revolutions_per_second = 1 / (sec_per_pulse * pulses_per_revolution);

     // Speed in meters per second (m/s)
     float speed_meters_per_sec = revolutions_per_second * wheel_circumference;

     right_prev_pulse_count = right_pulse_count;

     return speed_meters_per_sec;
}
