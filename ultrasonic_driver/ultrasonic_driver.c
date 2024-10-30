// Get readings from ultrasonic sensor

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"

int timeout = 26100;

absolute_time_t startTime;
absolute_time_t endTime;

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

uint64_t measureEchoPulse(uint trigPin, uint echoPin)
{
     uint64_t width = 0;
     startTime = get_absolute_time();

     while (gpio_get(echoPin) == 1)
     {
          endTime = get_absolute_time();
          if (absolute_time_diff_us(startTime, endTime) > timeout)
               return 0;
     }

     return absolute_time_diff_us(startTime, endTime);
}

double getDistance(uint trigPin, uint echoPin)
{
     uint64_t pulseLength = measureEchoPulse(trigPin, echoPin);
     printf("IR %llu us, ", pulseLength);
     double distance = (pulseLength / 29) * 0.5; // Calculate with higher precision
     printf("IR %f cm\n", distance);
     return distance;
}

uint64_t getPulse(uint trigPin, uint echoPin)
{
     gpio_put(trigPin, 1);
     sleep_us(10);
     gpio_put(trigPin, 0);

     uint64_t width = 0;

     while (gpio_get(echoPin) == 0)
          tight_loop_contents();
     absolute_time_t tempStartTime = get_absolute_time();
     while (gpio_get(echoPin) == 1)
     {
          width++;
          sleep_us(1);
          if (width > timeout)
               return 0;
     }
     absolute_time_t tempEndTime = get_absolute_time();

     return absolute_time_diff_us(tempStartTime, tempEndTime);
}

double getCm(uint trigPin, uint echoPin)
{
     uint64_t pulseLength = getPulse(trigPin, echoPin);
     printf("%llu us, ", pulseLength);
     double distance = pulseLength / 29 / 2; // Calculate with higher precision
     printf("%f cm\n", distance);
     return distance;
}