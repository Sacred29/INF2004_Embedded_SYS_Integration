#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/uart.h"

float wheel_diameter = 0.065;     // Wheel diameter in meters
float pulses_per_revolution = 20; // Adjust based on the number of marks or holes the IR sensor detects per wheel revolution

volatile uint32_t right_pulse_count = 0;        // Counts IR sensor pulses (wheel rotations)
volatile uint64_t right_last_pulse_time = 0;    // Time of the last IR sensor pulse
volatile uint64_t right_current_pulse_time = 0; // Time of the current IR sensor pulse
volatile float right_pulse_width = 0.0;

volatile float left_pulse_width = 0.0;
volatile uint32_t left_pulse_count = 0;        // Counts IR sensor pulses (wheel rotations)
volatile uint64_t left_last_pulse_time = 0;    // Time of the last IR sensor pulse
volatile uint64_t left_current_pulse_time = 0; // Time of the current IR sensor pulse

// Define GPIO pins Right Motor
#define RIGHT_PWM_PIN 2    // GP2 for PWM
#define RIGHT_DIR_PIN1 0   // GP0 for direction
#define RIGHT_DIR_PIN2 1   // GP1 for direction
#define RIGHT_SENSOR_PIN 4 // GPIO for the right IR sensor

// Define GPIO pins Left Motor
#define LEFT_PWM_PIN 8     // GP8 for PWM
#define LEFT_DIR_PIN1 6    // GP6 for direction
#define LEFT_DIR_PIN2 7    // GP7 for direction
#define LEFT_SENSOR_PIN 26 // GPIO for the left IR sensor

// Initialize button pin
const uint BTN_PIN = 21;
const uint BTN_PIN_INCREASE = 22;
const uint BTN_PIN_DECREASE = 20;

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

// Interrupt callback to count pulses from the IR sensor
void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == RIGHT_SENSOR_PIN)
    {
        printf("Detecting Right Pulse");
        // right_last_pulse_time = right_current_pulse_time; // Update last pulse time to the previous pulse time
        right_current_pulse_time = time_us_64(); // Get the current time in microseconds
        right_pulse_count++;                     // Increment the pulse count (each pulse represents part of a wheel rotation)
    }
    else if (gpio == LEFT_SENSOR_PIN)
    {
        printf("Detecting Left Pulse");
        // left_last_pulse_time = left_current_pulse_time; // Update last pulse time to the previous pulse time
        left_current_pulse_time = time_us_64(); // Get the current time in microseconds
        left_pulse_count++;                     // Increment the pulse count (each pulse represents part of a wheel rotation)
    }
}

// MARK: Test Codes
void wheel_encoder()
{
    static bool right_previous_state = true;
    static bool left_previous_state = true;

    bool right_current_state;
    bool left_current_state;

    // float pulse_width = 0.0;
    uint64_t start_time = time_us_64();

    while (1)
    {
        right_current_state = gpio_get(RIGHT_SENSOR_PIN);
        left_current_state = gpio_get(LEFT_SENSOR_PIN);

        uint64_t current_time = time_us_64();

        if (!right_previous_state && right_current_state)
        {
            if (right_pulse_count != 0)
            {
                // Calculate the wheel speed based on pulse timing
                right_pulse_width = (current_time - right_last_pulse_time) / 1000.0; // in milliseconds

                // Print the calculated wheel speed
                // printf("Right Pulse Width: %.2f ms\n", right_pulse_width);
            }

            // Update the last pulse time to the current time
            right_last_pulse_time = current_time;
            right_pulse_count++;
        }
        else if (!left_previous_state && left_current_state)
        {
            if (left_pulse_count != 0)
            {
                // Calculate the wheel speed based on pulse timing
                left_pulse_width = (current_time - left_last_pulse_time) / 1000.0; // in milliseconds

                // Print the calculated wheel speed
                // printf("Left Pulse Width: %.2f ms\n", left_pulse_width);
            }

            // Update the last pulse time to the current time
            left_last_pulse_time = current_time;
            left_pulse_count++;
        }

        if (difftime(current_time, start_time) >= 1000000)
        {
            break;
        }

        right_previous_state = right_current_state;
        left_previous_state = left_current_state;
    }

    // return pulse_width / 1000;
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
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65536));

    // Enable the PWM
    pwm_set_enabled(slice_num, true);
}

void set_speed(float duty_cycle, uint gpio_pin)
{
    pwm_set_gpio_level(gpio_pin, (uint16_t)(duty_cycle * 65535));
}

// Function to compute the control signal
float compute_pid(PIDController *pid, float setpoint, float current_value)
{

    float error = setpoint - current_value; // compute error

    pid->integral += error; // update integral term

    float derivative = error - pid->prev_error; // compute derivative term

    float control_signal = pid->Kp * error + pid->Ki * (pid->integral) + pid->Kd * derivative; // compute control signal

    pid->prev_error = error; // update previous error

    return control_signal;
}

int main()
{
    stdio_init_all();
    sleep_ms(5000);
    printf("Hello Motor Control\n");
    printf("Press and hold GP21 to change direction\n");
    printf("Press GP22 to increase speed\n");
    printf("Press GP20 to decrease speed\n");

    // Initialize GPIO pins for direction control of the right motor
    gpio_init(RIGHT_DIR_PIN1);
    gpio_init(RIGHT_DIR_PIN2);
    gpio_set_dir(RIGHT_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(RIGHT_DIR_PIN2, GPIO_OUT);

    // Initialize GPIO pins for direction control of the left motor
    gpio_init(LEFT_DIR_PIN1);
    gpio_init(LEFT_DIR_PIN2);
    gpio_set_dir(LEFT_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(LEFT_DIR_PIN2, GPIO_OUT);

    // Initialize GPIO pins for Speed Control
    gpio_init(BTN_PIN_INCREASE);
    gpio_set_dir(BTN_PIN_INCREASE, GPIO_IN);
    gpio_pull_up(BTN_PIN_INCREASE);

    gpio_init(BTN_PIN_DECREASE);
    gpio_set_dir(BTN_PIN_DECREASE, GPIO_IN);
    gpio_pull_up(BTN_PIN_DECREASE);

    // Initialize GPIO pin connected to the IR sensor (assume GPIO 2) for right motor
    gpio_init(4);
    gpio_set_dir(4, GPIO_IN);
    gpio_pull_up(4); // Enable pull-up resistor if necessary for your sensor

    // Initialize GPIO pin connected to the IR sensor (assume GPIO 26) for left motor
    gpio_init(26);
    gpio_set_dir(26, GPIO_IN);
    gpio_pull_up(26); // Enable pull-up resistor if necessary for your sensor

    // Set up PWM on GPIO2 and GPIO8
    setup_pwm(RIGHT_PWM_PIN, 100.0f, 0.5f); // 100 Hz frequency, 50% duty cycle
    setup_pwm(LEFT_PWM_PIN, 100.0f, 0.5f);  // 100 Hz frequency, 50% duty cycle

    // Initialize button
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    // PID Controller
    PIDController right_pid, left_pid;
    setup_pid(&right_pid, 1.0, 0.01, 0.02);
    setup_pid(&left_pid, 1.0, 0.01, 0.02);

    float setpoint = 120.0; // Desired position

    bool isReversed = false;

    // Control motor direction
    while (true)
    {
        // Check if button is pressed
        if (!gpio_get(BTN_PIN))
        {
            isReversed = !isReversed;
            printf("GP21 is pressed to reverse direction\n");
            sleep_ms(500);

            // Reverse direction
            if (isReversed)
            {
                gpio_put(RIGHT_DIR_PIN1, 1);
                gpio_put(RIGHT_DIR_PIN2, 0);

                gpio_put(LEFT_DIR_PIN1, 0);
                gpio_put(LEFT_DIR_PIN2, 1);
                // sleep_ms(2000);
                // tight_loop_contents();
            }
        }
        else
        {
            // Forward direction
            gpio_put(RIGHT_DIR_PIN1, 0);
            gpio_put(RIGHT_DIR_PIN2, 1);

            gpio_put(LEFT_DIR_PIN1, 1);
            gpio_put(LEFT_DIR_PIN2, 0);
            // sleep_ms(2000);
            // tight_loop_contents();
        }

        // // Speed Control
        // if (!gpio_get(BTN_PIN_INCREASE))
        // {
        //     printf("GP22 is pressed, Speed is increased\n");
        //     sleep_ms(500);
        //     set_speed(0.8f);
        // }
        // else if (!gpio_get(BTN_PIN_DECREASE))
        // {
        //     printf("GP20 is pressed, Speed is decreased\n");
        //     sleep_ms(500);
        //     set_speed(0.3f);
        // }

        sleep_ms(500); // To avoid button debouncing

        // Poll the IR sensors for wheel pulses
        wheel_encoder();

        // Measure the current pulse width for each wheel
        float current_right_pulse_width = right_pulse_width;
        float current_left_pulse_width = left_pulse_width;

        // Compute the control signal for the right motor
        float right_control_signal = compute_pid(&right_pid, setpoint, current_right_pulse_width);

        // Compute the control signal for the left motor
        float left_control_signal = compute_pid(&left_pid, setpoint, current_left_pulse_width);

        // Print the calculated wheel speed
        printf("BEFORE  Right Pulse Width: %.2f\n", right_pulse_width);
        printf("BEFORE  Left Pulse Width: %.2f\n", left_pulse_width);

        // Print the control signal
        printf("Right Control Signal: %.2f\n", right_control_signal);
        printf("Left Control Signal: %.2f\n", left_control_signal);

        // Update the motor speed based on the control signal
        float right_duty_cycle = fmin(fmax(right_control_signal, 0.25), 0.5); // duty cycle is between 0 and 0.5
        float left_duty_cycle = fmin(fmax(left_control_signal, 0.25), 0.5);

        printf("Right Duty Cycle: %.2f\n", right_duty_cycle);
        printf("Left Duty Cycle: %.2f\n", left_duty_cycle);

        set_speed(right_duty_cycle, RIGHT_PWM_PIN); // Replace with your right motor speed function
        set_speed(left_duty_cycle, LEFT_PWM_PIN);   // Replace with your left motor speed function

        sleep_ms(100); // Adjust polling rate as needed
    }

    return 0;
}