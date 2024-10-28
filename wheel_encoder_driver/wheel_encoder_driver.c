#include <stdio.h>
#include <time.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

float wheel_diameter = 0.065;     // Wheel diameter in meters
float pulses_per_revolution = 20; // Adjust based on the number of marks or holes the IR sensor detects per wheel revolution// Calculate the wheel speed in meters per second (m/s)

void setupWheelEncoderPins(uint inputPin)
{
     gpio_init(inputPin);
     gpio_set_dir(inputPin, GPIO_IN);
     gpio_pull_up(inputPin);
}

float calculate_wheel_speed(float pulse_width)
{
     // Time between pulses (in seconds)
     float time_between_pulses = pulse_width * 0.000001f; // Convert microseconds to seconds

     if (time_between_pulses > 0)
     {
          // Wheel circumference
          float wheel_circumference = 3.14159 * wheel_diameter;

          // The number of complete revolution per second
          float revolutions_per_second = 1 / (time_between_pulses * pulses_per_revolution);

          // Speed in meters per second (m/s)
          float speed_meters_per_sec = revolutions_per_second * wheel_circumference;

          return speed_meters_per_sec;
     }
     else
     {
          return 0.0; // If no pulse detected, speed is 0
     }
}

float wheel_encoder(uint gpio)
{
     uint32_t pulse_count = 0;

     bool current_state;
     bool previous_state = false;

     float total_pulse_width = 0.0;
     float current_pulse_width = 0.0;

     uint64_t start_time = time_us_64();
     uint64_t current_pulse_time = 0;
     uint64_t last_pulse_time = 0;

     while (1)
     {
          current_state = gpio_get(gpio);
          current_pulse_time = time_us_64();

          if (current_state && !previous_state)
          {
               if (pulse_count >= 1)
               {
                    // Calculate the wheel speed based on pulse timing
                    current_pulse_width = current_pulse_time - last_pulse_time;
                    total_pulse_width += current_pulse_width;
               }

               // Update the last pulse time to the current time
               last_pulse_time = current_pulse_time;
               pulse_count++;

               // Print the calculated wheel speed
               printf("1 Pulse Width: %.2f us\n", current_pulse_width);
               printf("Total Pulse Width: %.2f us, Pulse Count: %i\n", total_pulse_width, pulse_count);
          }

          if ((current_pulse_time - start_time) >= 1000000)
          {
               break;
          }

          previous_state = current_state;
     }

     float wheel_speed = calculate_wheel_speed(total_pulse_width / (pulse_count - 1));
     return wheel_speed;
}