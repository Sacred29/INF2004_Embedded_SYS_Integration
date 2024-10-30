#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "pico/stdlib.h"

void setup_pwm(uint gpio, float freq, float duty_cycle);
void set_speed(float duty_cycle, uint gpio_pin);
void setup_motor_direction(uint dir_pin1, uint dir_pin2);
void set_motor_direction(bool reverse, uint dir_pin1, uint dir_pin2);
void initialize_speed_change_button(uint BTN_PIN);

#endif // MOTOR_CONTROL_H
