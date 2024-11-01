#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "wheel_encoder_driver.h"
#include "ultrasonic_driver.h"

// Define GPIO pins Right Motor
#define RIGHT_PWM_PIN 2    // GP2 for PWM
#define RIGHT_DIR_PIN1 0   // GP0 for direction
#define RIGHT_DIR_PIN2 1   // GP1 for direction
#define RIGHT_SENSOR_PIN 4 // GPIO for the right IR sensor

// Define GPIO pins Left Motor
#define LEFT_PWM_PIN 10    // GP8 for PWM
#define LEFT_DIR_PIN1 6    // GP6 for direction
#define LEFT_DIR_PIN2 7    // GP7 for direction
#define LEFT_SENSOR_PIN 26 // GPIO for the left IR sensor

// Define GPIO pins Left Motor
#define TRIG_PIN 8 // GP8 for Trigger Pin
#define ECHO_PIN 9 // GP9 for Echo Pin

// Control constants
#define BASELINE_DC 0.50
#define TARGET_SPEED 19.0
#define ADJUSTMENT_FACTOR 0.04

// Initialize button pin
const uint BTN_PIN_START = 22;

float left_speed = 0.0;
float right_speed = 0.0;

// float timer_value = 500000;
// float timer_value = 100000;
float timer_value = 250000;

// PID Controller
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
} PIDController;

void setup_pid(PIDController *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
}
// Function to compute the control signal
float compute_pid(PIDController *pid, float setpoint, float current_value)
{

    float error = setpoint - current_value; // compute error

    pid->integral += error; // compute integral term

    float derivative = error - pid->prev_error; // compute derivative term

    float control_signal = pid->Kp * error + pid->Ki * (pid->integral) + pid->Kd * derivative; // compute control signal

    pid->prev_error = error; // update previous error

    return control_signal;
}

// Function to set up the PWM
void setup_pwm(uint gpio, float freq, float duty_cycle)
{
    // Set the GPIO function to PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Calculate the PWM frequency and set the PWM wrap value
    float clock_freq = 125000000.0f;                // Default Pico clock frequency in Hz
    uint32_t divider = clock_freq / (freq * 65536); // Compute divider for given frequency
    pwm_set_clkdiv(slice_num, divider);

    // Set the PWM wrap value (maximum count value)
    pwm_set_wrap(slice_num, 65535); // 16-bit counter (0 - 65535)

    // Set the duty cycle
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65535));

    // Enable the PWM
    pwm_set_enabled(slice_num, true);
}
float left_distance = 0.0;
float right_distance = 0.0;

void set_speed(float duty_cycle, uint gpio_pin)
{
    pwm_set_gpio_level(gpio_pin, (uint16_t)(duty_cycle * 65535));
}

// Global PID controllers and moving state
PIDController right_pid, left_pid;
bool isMoving = false; // Tracks movement state

bool repeating_timer_callback(__unused struct repeating_timer *t)
{
    // left_speed = getLeftWheelSpeed(timer_value);
    // right_speed = getRightWheelSpeed(timer_value);
    // printf("Left Wheel Speed: %.2f cm/s\n", left_speed);
    // printf("Right Wheel Speed: %.2f cm/s\n", right_speed);
    // return true;

    // static int timer_count = 0; // Counter to keep track of elapsed time

    // printf("repeating_timer_callback triggered, isMoving = %d\n", isMoving);
    if (isMoving)
    {
        // Read wheel speeds
        getLeftWheelInfo(&left_speed, &left_distance);
        getRightWheelInfo(&right_speed, &right_distance);

        // Display current speeds for debugging
        printf("Left Wheel Speed: %.5f cm/s\n", left_speed);
        printf("Right Wheel Speed: %.5f cm/s\n", right_speed);

        // // Compute PID control signals for each wheel
        // float right_control_signal = compute_pid(&right_pid, TARGET_SPEED, right_speed);
        // float left_control_signal = compute_pid(&left_pid, TARGET_SPEED, left_speed);

        // // Calculate and apply new duty cycles
        // float right_duty_cycle = fmin(fmax(BASELINE_DC + (right_control_signal * ADJUSTMENT_FACTOR), 0.0), 1.0);
        // float left_duty_cycle = fmin(fmax(BASELINE_DC + (left_control_signal * ADJUSTMENT_FACTOR), 0.0), 1.0);

        // // set_speed(right_duty_cycle, RIGHT_PWM_PIN);
        // // set_speed(left_duty_cycle, LEFT_PWM_PIN);
        set_speed(0.80, RIGHT_PWM_PIN);
        set_speed(0.80, LEFT_PWM_PIN);
        printf("This is speed 80%\n");

        // timer_count++; // Increment the counter each time the callback is called

        // // For the first 10 seconds, run at 50% duty cycle
        // if (timer_count <= 20)
        // { // Assuming the callback runs every 500 ms
        //     set_speed(0.5, RIGHT_PWM_PIN);
        //     set_speed(0.5, LEFT_PWM_PIN);
        //     printf("This is speed 50%%\n");
        // }
        // // After 10 seconds, switch to 80% duty cycle
        // else
        // {
        //     set_speed(0.8, RIGHT_PWM_PIN);
        //     set_speed(0.8, LEFT_PWM_PIN);
        //     printf("This is speed 80%%\n");
        // }

        // // Optional: Continue PID calculations if desired
        // left_speed = getLeftWheelSpeed(timer_value);
        // right_speed = getRightWheelSpeed(timer_value);

        // printf("Left Wheel Speed: %.5f cm/s\n", left_speed);
        // printf("Right Wheel Speed: %.5f cm/s\n", right_speed);
    }

    return true; // Keep the timer running
}

// GPIO and PWM setup functions
void setup_gpio_pins()
{
    gpio_init(BTN_PIN_START);
    gpio_set_dir(BTN_PIN_START, GPIO_IN);
    gpio_pull_up(BTN_PIN_START);

    // Initialize and set direction control pins for the right motor
    gpio_init(RIGHT_DIR_PIN1);
    gpio_init(RIGHT_DIR_PIN2);
    gpio_set_dir(RIGHT_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(RIGHT_DIR_PIN2, GPIO_OUT);
    gpio_put(RIGHT_DIR_PIN1, 0); // Set initial direction for right motor
    gpio_put(RIGHT_DIR_PIN2, 1);

    // Initialize and set direction control pins for the left motor
    gpio_init(LEFT_DIR_PIN1);
    gpio_init(LEFT_DIR_PIN2);
    gpio_set_dir(LEFT_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(LEFT_DIR_PIN2, GPIO_OUT);
    gpio_put(LEFT_DIR_PIN1, 0); // Set initial direction for left motor
    gpio_put(LEFT_DIR_PIN2, 1);
}

void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == RIGHT_SENSOR_PIN)
    {
        rightWheelPulseCounting();
    }
    if (gpio == LEFT_SENSOR_PIN)
    {
        leftWheelPulseCounting();
    }
    if (gpio == ECHO_PIN)
    {
        if (getDistance(TRIG_PIN, ECHO_PIN) < 10)
        {
            printf("Obstacle detected\n");
            // Stop the motors
            // set_speed(0.0, RIGHT_PWM_PIN);
            // set_speed(0.0, LEFT_PWM_PIN);
        }
    }
}

// Define initial duty cycle, target duty cycle, and increment step
const float initial_duty_cycle = 0.4; // Starting point, lower than baseline
const float target_duty_cycle = 0.7;  // Baseline duty cycle
const float ramp_increment = 0.05;    // Increment step for ramp-up
const uint ramp_delay_ms = 100;       // Delay between increments in milliseconds

// Function to ramp up duty cycle gradually
void ramp_up_duty_cycle(float initial_duty_cycle, float target_duty_cycle, uint gpio_pin_left, uint gpio_pin_right)
{
    float current_duty_cycle = initial_duty_cycle;

    while (current_duty_cycle < target_duty_cycle)
    {
        set_speed(current_duty_cycle, gpio_pin_left);  // Apply current duty cycle to left motor
        set_speed(current_duty_cycle, gpio_pin_right); // Apply current duty cycle to right motor
        // printf("Ramping up: Duty Cycle = %.2f\n", current_duty_cycle);
        current_duty_cycle = fmin(current_duty_cycle + ramp_increment, target_duty_cycle);
        sleep_ms(ramp_delay_ms); // Delay to control ramp-up speed
    }

    // Ensure both wheels reach the exact target duty cycle
    set_speed(target_duty_cycle, gpio_pin_left);
    set_speed(target_duty_cycle, gpio_pin_right);
    printf("Ramp-up complete: Final Duty Cycle = %.2f\n", target_duty_cycle);
}
int main()
{
    stdio_init_all();
    sleep_ms(5000);
    printf("Hello Motor Control\n");
    printf("Press and hold GP21 to change direction\n");
    printf("Press GP22 to start\n");

    setup_gpio_pins();
    setupWheelEncoderPins(LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN);
    // gpio_set_irq_enabled_with_callback(RIGHT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    // gpio_set_irq_enabled(LEFT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled_with_callback(RIGHT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(LEFT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true);

    // Initialize PID controllers for both wheels with appropriate values
    setup_pid(&left_pid, 0, 0, 0.0);  // Adjust Kp, Ki, Kd as needed for left motor
    setup_pid(&right_pid, 0, 0, 0.0); // Adjust Kp, Ki, Kd as needed for right motor

    struct repeating_timer timer;
    add_repeating_timer_us(timer_value, repeating_timer_callback, NULL, &timer);

    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);
    gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_RISE, true);

    // Set up PWM on GPIO2 and GPIO8
    setup_pwm(RIGHT_PWM_PIN, 100.0f, 0.0f); // 100 Hz frequency, 50% duty cycle
    setup_pwm(LEFT_PWM_PIN, 100.0f, 0.0f);  // 100 Hz frequency, 50% duty cycle

    // Control motor direction
    while (true)
    {
        // Check if button is pressed
        // Wait until GP22 (BTN_PIN_INCREASE) is pressed to start moving
        if (!isMoving && !gpio_get(BTN_PIN_START))
        {
            isMoving = true;
            printf("GP22 pressed, starting movement\n");
            printf("isMoving = %d\n", isMoving);
            sleep_ms(500); // Debounce delay

            // Perform ramp-up for both motors simultaneously
            // ramp_up_duty_cycle(initial_duty_cycle, target_duty_cycle, LEFT_PWM_PIN, RIGHT_PWM_PIN);

            // Now motors have reached target speed, and PID control can take over
        }

        // Print the state of isMoving in the main loop for debugging
        // printf("Main loop: isMoving = %d\n", isMoving);
    }
    return 0;
}