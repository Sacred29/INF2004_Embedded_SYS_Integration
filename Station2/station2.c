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

float left_speed = 0.0;
float right_speed = 0.0;

float timer_value = 500000;


// Define GPIO pins Right Motor
#define RIGHT_PWM_PIN 2    // GP2 for PWM
#define RIGHT_DIR_PIN1 0   // GP0 for direction
#define RIGHT_DIR_PIN2 1   // GP1 for direction
#define RIGHT_SENSOR_PIN 4 // GPIO for the right IR sensor

// Define GPIO pins Left Motor
#define LEFT_PWM_PIN 10     // GP8 for PWM
#define LEFT_DIR_PIN1 6    // GP6 for direction
#define LEFT_DIR_PIN2 7    // GP7 for direction
#define LEFT_SENSOR_PIN 26 // GPIO for the left IR sensor

// Initialize button pin
const uint BTN_PIN = 21;
const uint BTN_PIN_INCREASE = 22;
const uint BTN_PIN_DECREASE = 20;

// Define Baseline Duty Cycle Values
#define BASELINE_DC  0.5
#define TARGET_SPEED 18 // Target speed in cm/s

// PID Controller
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
} PIDController;

typedef struct {
    bool *isMoving;
    PIDController *right_pid;
    PIDController *left_pid;
} SpeedAdjustData;


void forward_direction ();
void initialize_direction_control();
void initialize_speed_control_buttons();
void wheel_encoder_interrupt();
bool pid_speed_adjust_function(struct repeating_timer *t);
void setup_pid(PIDController *pid, float Kp, float Ki, float Kd);
void setup_pwm(uint gpio, float freq, float duty_cycle);
void set_speed(float duty_cycle, uint gpio_pin);
float compute_pid(PIDController *pid, float setpoint, float current_value);
void gpio_callback(uint gpio, uint32_t events);
bool repeating_timer_callback(__unused struct repeating_timer *t);


int main()
{
    stdio_init_all();
    sleep_ms(5000);
    printf("Hello Motor Control\n");
    printf("Press and hold GP21 to change direction\n");
    printf("Press GP22 to start\n");

    initialize_direction_control();

    initialize_speed_control_buttons();

    wheel_encoder_interrupt();
    
    struct repeating_timer timer;
    add_repeating_timer_us(timer_value, repeating_timer_callback, NULL, &timer);

    // Set up PWM on GPIO2 and GPIO8
    setup_pwm(RIGHT_PWM_PIN, 100.0f, 0.0f); // 100 Hz frequency, 50% duty cycle
    setup_pwm(LEFT_PWM_PIN, 100.0f, 0.0f);  // 100 Hz frequency, 50% duty cycle

    
    // PID Controller
    PIDController right_pid, left_pid;
    setup_pid(&right_pid, 0.6, 0.005, 0);
    setup_pid(&left_pid, 0.45, 0.005, 0);

    

    bool isMoving = false;

    // Forward direction
    forward_direction();

    SpeedAdjustData speed_adjust_data = {&isMoving, &right_pid, &left_pid};
    struct repeating_timer pid_speed_adjust_timer;
    add_repeating_timer_ms(500, pid_speed_adjust_function, &speed_adjust_data, &pid_speed_adjust_timer);

    // Control motor direction
    while (true)
    {
        // Check if button is pressed
        // Wait until GP22 (BTN_PIN_INCREASE) is pressed to start moving
        if (!isMoving && !gpio_get(BTN_PIN_INCREASE))
        {
            isMoving = true;
            printf("GP22 pressed, starting movement\n");
            sleep_ms(550); // Debounce delay

            // Set initial duty cycle to 0.5 for both motors to start moving
            set_speed(0.5, RIGHT_PWM_PIN);
            set_speed(0.5, LEFT_PWM_PIN);
        }

        if (isMoving)
        {
            // Reduce adjustment factor to reduce control signal impact further
            float adjustment_factor = 0.04;

            float right_duty_cycle = BASELINE_DC + (compute_pid(&right_pid, TARGET_SPEED , right_speed) * adjustment_factor);
            float left_duty_cycle = BASELINE_DC + (compute_pid(&left_pid, TARGET_SPEED, left_speed) * adjustment_factor);

            // set_speed(right_duty_cycle, RIGHT_PWM_PIN);
            // set_speed(left_duty_cycle, LEFT_PWM_PIN);

            set_speed(0.6, RIGHT_PWM_PIN);
            set_speed(0.6, LEFT_PWM_PIN);
        }

        sleep_ms(500); // Adjust polling rate as needed
    }

    return 0;
}
void forward_direction () {
    gpio_put(RIGHT_DIR_PIN1, 0);
    gpio_put(RIGHT_DIR_PIN2, 1);

    gpio_put(LEFT_DIR_PIN1, 0);
    gpio_put(LEFT_DIR_PIN2, 1);
}

void initialize_direction_control() {
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
}
void initialize_speed_control_buttons () {
// Initialize GPIO pins for Speed Control
    gpio_init(BTN_PIN_INCREASE);
    gpio_set_dir(BTN_PIN_INCREASE, GPIO_IN);
    gpio_pull_up(BTN_PIN_INCREASE);

    gpio_init(BTN_PIN_DECREASE);
    gpio_set_dir(BTN_PIN_DECREASE, GPIO_IN);
    gpio_pull_up(BTN_PIN_DECREASE);
    
    // Initialize button
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

}
void wheel_encoder_interrupt() {
    setupWheelEncoderPins(LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN);
    gpio_set_irq_enabled_with_callback(RIGHT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(LEFT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true);
}

bool pid_speed_adjust_function(struct repeating_timer *t) {
    SpeedAdjustData *data = (SpeedAdjustData *)t->user_data; // Retrieve data passed in

    if (*(data->isMoving))
    {
        float adjustment_factor = 0.04;
        float right_duty_cycle = BASELINE_DC + (compute_pid(data->right_pid, TARGET_SPEED, right_speed) * adjustment_factor);
        float left_duty_cycle = BASELINE_DC + (compute_pid(data->left_pid, TARGET_SPEED, left_speed) * adjustment_factor);

        set_speed(right_duty_cycle, RIGHT_PWM_PIN);
        set_speed(left_duty_cycle, LEFT_PWM_PIN);
    }
    return true;
}

void setup_pid(PIDController *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
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

void set_speed(float duty_cycle, uint gpio_pin)
{
    pwm_set_gpio_level(gpio_pin, (uint16_t)(duty_cycle * 65535));
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
}

bool repeating_timer_callback(__unused struct repeating_timer *t)
{
    left_speed = getLeftWheelSpeed(timer_value);
    right_speed = getRightWheelSpeed(timer_value);
    printf("Left Wheel Speed: %.2f cm/s\n", left_speed);
    printf("Right Wheel Speed: %.2f cm/s\n", right_speed);
    return true;
}
