#include <stdio.h>
#include <time.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define WHEEL_DIAMETER 6.5       // Wheel diameter in centimeters
#define PULSES_PER_REVOLUTION 20 // Adjust based on the number of marks or holes the IR sensor detects per wheel revolution// Calculate the wheel speed in meters per second (m/s)

float left_pulse_count = 0;
float right_pulse_count = 0;
float left_prev_pulse_count = 0;
float right_prev_pulse_count = 0;
absolute_time_t left_last_recorded_time;
absolute_time_t right_last_recorded_time;

void setupWheelEncoderPins(uint left_wheel_pin, uint right_wheel_pin)
{
     gpio_init(left_wheel_pin);
     gpio_set_dir(left_wheel_pin, GPIO_IN);
     gpio_pull_up(left_wheel_pin);

     gpio_init(right_wheel_pin);
     gpio_set_dir(right_wheel_pin, GPIO_IN);
     gpio_pull_up(right_wheel_pin);

     left_last_recorded_time = get_absolute_time();
     right_last_recorded_time = get_absolute_time();
}

void leftWheelPulseCounting()
{
     left_pulse_count++;
}

void rightWheelPulseCounting()
{
     right_pulse_count++;
}

void reset_distance_traveled(float *left_distance, float *right_distance)
{
     *left_distance = 0.0;
     *right_distance = 0.0;
     printf("Distance traveled reset to zero.\n");
}

void getLeftWheelInfo(float *left_wheel_speed, float *left_distance_travel)
{
     // Number of Pulses within a fixed timed
     float left_current_pulse_count = left_pulse_count - left_prev_pulse_count;

     // The duration of recording the pulses
     absolute_time_t left_current_time = get_absolute_time();
     int64_t left_recorded_duration = absolute_time_diff_us(left_last_recorded_time, left_current_time);

     // Calculate the seconds per pulse
     float left_recorded_duration_seconds = left_recorded_duration * 0.000001f;
     float seconds_per_pulse = left_recorded_duration_seconds / left_current_pulse_count;

     // Circumference of the Wheel
     float wheel_circumference = 3.14159 * WHEEL_DIAMETER;

     // The number of complete revolution per second
     float revolutions_per_second = 1 / (seconds_per_pulse * PULSES_PER_REVOLUTION);

     // Speed in cemtimeters per second (cm/s)
     float speed_cemtimeters_per_sec = revolutions_per_second * wheel_circumference;
     *left_wheel_speed = speed_cemtimeters_per_sec;

     // Distance travelled by the left wheel
     float left_wheel_distance_travelled = speed_cemtimeters_per_sec * left_recorded_duration_seconds;
     *left_distance_travel += left_wheel_distance_travelled;

     left_prev_pulse_count = left_pulse_count;
     left_last_recorded_time = left_current_time;
}

void getRightWheelInfo(float *right_wheel_speed, float *right_distance_travel)
{
     // Number of Pulses within a fixed time
     float right_current_pulse_count = right_pulse_count - right_prev_pulse_count;

     // The duration of recording the pulses
     absolute_time_t right_current_time = get_absolute_time();
     int64_t right_recorded_duration = absolute_time_diff_us(right_last_recorded_time, right_current_time);

     // Calculate the seconds per pulse
     float right_recorded_duration_seconds = right_recorded_duration * 0.000001f;
     float seconds_per_pulse = right_recorded_duration_seconds / right_current_pulse_count;

     // Circumference of the Wheel
     float wheel_circumference = 3.14159 * WHEEL_DIAMETER;

     // The number of complete revolutions per second
     float revolutions_per_second = 1 / (seconds_per_pulse * PULSES_PER_REVOLUTION);

     // Speed in centimeters per second (cm/s)
     float speed_centimeters_per_sec = revolutions_per_second * wheel_circumference;
     *right_wheel_speed = speed_centimeters_per_sec;

     // Distance traveled by the right wheel
     float right_wheel_distance_traveled = speed_centimeters_per_sec * right_recorded_duration_seconds;
     *right_distance_travel += right_wheel_distance_traveled;

     right_prev_pulse_count = right_pulse_count;
     right_last_recorded_time = right_current_time;
}