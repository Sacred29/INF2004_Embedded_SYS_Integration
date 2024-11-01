// Get readings from ultrasonic sensor

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"

int timeout = 2900;

void setupUltrasonicPins(uint trigPin, uint echoPin)
{
     gpio_init(trigPin);
     gpio_init(echoPin);
     gpio_set_dir(trigPin, GPIO_OUT);
     gpio_set_dir(echoPin, GPIO_IN);
}

void sentTrigPulse(uint trigPin, uint echoPin)
{
     gpio_put(trigPin, 1);
     sleep_us(10);
     gpio_put(trigPin, 0);
}

int64_t measureEchoPulse(uint trigPin, uint echoPin)
{
     absolute_time_t startTime = get_absolute_time();
     absolute_time_t endTime;

     while (gpio_get(echoPin) == 1)
     {
          endTime = get_absolute_time();
          if (absolute_time_diff_us(startTime, endTime) > timeout)
               return -1;
     }

     return absolute_time_diff_us(startTime, endTime);
}

double getDistance(uint trigPin, uint echoPin)
{
     int64_t pulseLength = measureEchoPulse(trigPin, echoPin);

     if (pulseLength == -1)
     {
          return pulseLength;
     }

     double distance = (pulseLength / 29) * 0.5;
     printf("Echo: %lld us, Distance: %f cm\n", pulseLength, distance);

     return distance;
}