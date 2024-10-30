#include "motor_control.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

void setup_pwm(uint gpio, float freq, float duty_cycle) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    float clock_freq = 125000000.0f;
    uint32_t divider = clock_freq / (freq * 65536);
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, 65535);
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65536));
    pwm_set_enabled(slice_num, true);
}

void set_speed(float duty_cycle, uint gpio_pin) {
    pwm_set_gpio_level(gpio_pin, (uint16_t)(duty_cycle * 65535));
}

void setup_motor_direction(uint dir_pin1, uint dir_pin2) {
    gpio_init(dir_pin1);
    gpio_init(dir_pin2);
    gpio_set_dir(dir_pin1, GPIO_OUT);
    gpio_set_dir(dir_pin2, GPIO_OUT);
}

void set_motor_direction(bool reverse, uint dir_pin1, uint dir_pin2) {
    gpio_put(dir_pin1, reverse ? 1 : 0);
    gpio_put(dir_pin2, reverse ? 0 : 1);
}

void initialize_speed_change_button(uint BTN_PIN) {
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);
}