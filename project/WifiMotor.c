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

// FREERTOS
#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/ip4_addr.h" // Maybe need (need to test)
#include "lwip/sockets.h"  // lwIP socket API for networking
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

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
#define BASELINE_DC 0.70
#define TARGET_SPEED 20.0
#define ADJUSTMENT_FACTOR 0.02
#define PULSES_PER_REVOLUTION 20 // Define the constant with an appropriate value

// FREERTOS
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define configMINIMAL_STACK_SIZE (2048)
#define configCHECK_FOR_STACK_OVERFLOW 2 // Enable stack overflow checking
#define SERVER_IP "172.20.10.7"          // PC/Server IP address
#define SERVER_PORT 65439                // Port no. the server is listening on
#define WIFI_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define WIFI_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 10240) // Increased stack size for FreeRTOS task
#define MOTOR_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define MOTOR_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 10240) // Increased stack size for FreeRTOS task

// Message buffer
MessageBufferHandle_t xMotorSpeedMessageBuffer;
#define WIFI_MESSAGE_BUFFER_SIZE (60)

// Initialize button pin
const uint BTN_PIN_START = 22;

float left_speed = 0.0;
float right_speed = 0.0;

float left_distance = 0.0;
float right_distance = 0.0;

// float timer_value = 500000;
// float timer_value = 100000;
float timer_value = 250000;

absolute_time_t start_time, end_time, turn_start_time, pause_start_time;
bool isDetected = false;
bool isTurning = false;
bool isPostTurnPause = false;

// Define initial duty cycle, target duty cycle, and increment step
const float initial_duty_cycle = 0.5; // Starting point, lower than baseline
const float target_duty_cycle = 0.7;  // Baseline duty cycle
const float ramp_increment = 0.05;    // Increment step for ramp-up
const uint ramp_delay_ms = 100;       // Delay between increments in milliseconds

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

    float error = setpoint - current_value;                                                    // compute error
    pid->integral += error;                                                                    // compute integral term
    float derivative = error - pid->prev_error;                                                // compute derivative term
    float control_signal = pid->Kp * error + pid->Ki * (pid->integral) + pid->Kd * derivative; // compute control signal
    pid->prev_error = error;                                                                   // update previous error
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

void set_speed(float duty_cycle, uint gpio_pin)
{
    pwm_set_gpio_level(gpio_pin, (uint16_t)(duty_cycle * 65535));
}

// Global PID controllers and moving state
PIDController right_pid, left_pid;
bool isMoving = false; // Tracks movement state

// MARK: Repeating Timer Callback
bool repeating_timer_callback(__unused struct repeating_timer *t)
{
    if (isDetected && !isTurning && !isPostTurnPause && !isMoving)
    {
        // printf("Obstacle detected, waiting for 3 seconds...\n");
        end_time = get_absolute_time();
        uint64_t time_diff = absolute_time_diff_us(start_time, end_time);
        if (time_diff > 3000000) // 3 seconds in microseconds
        {
            isDetected = false;
            turn_start_time = get_absolute_time();

            // Start right-angle turn
            gpio_put(RIGHT_DIR_PIN1, 1);
            gpio_put(RIGHT_DIR_PIN2, 0);
            gpio_put(LEFT_DIR_PIN1, 0);
            gpio_put(LEFT_DIR_PIN2, 1);

            set_speed(1.0, RIGHT_PWM_PIN);
            set_speed(1.0, LEFT_PWM_PIN);

            // Set isTurning to true to indicate the turn is in progress
            isTurning = true;

            // printf("Starting right-angle turn...\n");
        }
    }

    if (isTurning)
    {
        uint64_t turn_time_diff = absolute_time_diff_us(turn_start_time, get_absolute_time()) / 1000;
        if (turn_time_diff >= 275) // Adjust duration for a 90-degree turn (e.g., 500 ms)
        {
            // Stop the motors after the turn
            set_speed(0, RIGHT_PWM_PIN);
            set_speed(0, LEFT_PWM_PIN);

            // Reset distance traveled for the next phase by calling the reset function
            reset_distance_traveled(&left_distance, &right_distance);

            isTurning = false;
            isPostTurnPause = true; // Move to post-turn pause phase
            pause_start_time = get_absolute_time();
            // printf("Turn completed, pausing for 3 seconds...\n");
        }
    }

    // 3. Post-Turn Pause (1.5 seconds)
    if (isPostTurnPause)
    {
        uint64_t pause_time_diff = absolute_time_diff_us(pause_start_time, get_absolute_time()) / 1000;
        if (pause_time_diff >= 1500) // 3 seconds in milliseconds
        {
            isPostTurnPause = false;
            isMoving = true; // Move to straight-line movement phase
            // printf("Pause complete, starting straight-line movement...\n");

            // Set direction for forward movement
            gpio_put(RIGHT_DIR_PIN1, 0);
            gpio_put(RIGHT_DIR_PIN2, 1);
            gpio_put(LEFT_DIR_PIN1, 0);
            gpio_put(LEFT_DIR_PIN2, 1);

            // // Start moving forward at initial speed (PID will adjust)
            set_speed(0.75, RIGHT_PWM_PIN);
            set_speed(0.75, LEFT_PWM_PIN);

            // can also do the ramp-up here
            // ramp_up_duty_cycle(initial_duty_cycle, target_duty_cycle, LEFT_PWM_PIN, RIGHT_PWM_PIN);
        }
    }

    // printf("repeating_timer_callback triggered, isMoving = %d\n", isMoving);
    if (isMoving)
    {
        // Read wheel speeds
        getLeftWheelInfo(&left_speed, &left_distance);
        getRightWheelInfo(&right_speed, &right_distance);

        // // Display current speeds for debugging
        // printf("Left Wheel Speed: %.5f cm/s, %.2f\n", left_speed, left_distance);
        // printf("Right Wheel Speed: %.5f cm/s, %.2f\n", right_speed, right_distance);

        // Compute PID control signals for each wheel
        // The PID controller adjusts the motor speed to maintain the target speed
        float right_control_signal = compute_pid(&right_pid, TARGET_SPEED, right_speed);
        float left_control_signal = compute_pid(&left_pid, TARGET_SPEED, left_speed);
        // printf("Left Control Signal: %.2f\n", left_control_signal);
        // printf("Right Control Signal: %.2f\n", right_control_signal);

        // Calculate and apply new duty cycles
        float right_duty_cycle = BASELINE_DC + (right_control_signal * ADJUSTMENT_FACTOR);
        float left_duty_cycle = BASELINE_DC + (left_control_signal * ADJUSTMENT_FACTOR);
        // float right_duty_cycle = BASELINE_DC + right_control_signal;
        // float left_duty_cycle = BASELINE_DC + left_control_signal;
        // printf("Left Duty Cycle: %.2f\n", left_duty_cycle);
        // printf("Right Duty Cycle: %.2f\n\n", right_duty_cycle);

        set_speed(right_duty_cycle, RIGHT_PWM_PIN);
        set_speed(left_duty_cycle, LEFT_PWM_PIN);
        // set_speed(0.7, RIGHT_PWM_PIN);
        // set_speed(0.7, LEFT_PWM_PIN);

        // Check if the robot has moved a certain distance
        if (left_distance >= 90 || right_distance >= 90)
        {
            set_speed(0, RIGHT_PWM_PIN);
            set_speed(0, LEFT_PWM_PIN);
            isMoving = false;
            // Display current speeds for debugging
            printf("STOPPPP\n");
        }
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

// MARK: GPIO Callback
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
    if (gpio == ECHO_PIN && !isTurning && !isPostTurnPause) // Avoid repeated detections during turn or pause
    {
        double object_distance = getDistance(TRIG_PIN, ECHO_PIN);
        if (object_distance <= 12 && object_distance != -1 && !isDetected)
        {
            printf("Distance: %.2f cm\n", object_distance);
            printf("Obstacle detected\n");
            isDetected = true;
            // Stop the motors
            set_speed(0.0, RIGHT_PWM_PIN);
            set_speed(0.0, LEFT_PWM_PIN);
            isMoving = false;

            start_time = get_absolute_time();
        }
    }
}

// Function to ramp up duty cycle gradually
void ramp_up_duty_cycle(float initial_duty_cycle, float target_duty_cycle, uint gpio_pin_left, uint gpio_pin_right)
{
    float current_duty_cycle = initial_duty_cycle;

    while (current_duty_cycle < target_duty_cycle)
    {
        set_speed(current_duty_cycle, gpio_pin_left);  // Apply current duty cycle to left motor
        set_speed(current_duty_cycle, gpio_pin_right); // Apply current duty cycle to right motor
        printf("Ramping up: Duty Cycle = %.2f\n", current_duty_cycle);
        current_duty_cycle = fmin(current_duty_cycle + ramp_increment, target_duty_cycle);
        sleep_ms(ramp_delay_ms); // Delay to control ramp-up speed
    }

    // Ensure both wheels reach the exact target duty cycle
    set_speed(target_duty_cycle, gpio_pin_left);
    set_speed(target_duty_cycle, gpio_pin_right);
    printf("Ramp-up complete: Final Duty Cycle = %.2f\n", target_duty_cycle);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *wifi_task) // Function to check if stack overflow occurs
{
    printf("Stack overflow in task: %s\n", wifi_task);
    while (1)
        ; // Trap the error
}

// MARK:Wi-Fi transmission task
void wifi_task(__unused void *params)
{
    struct sockaddr_in server_addr;
    char message[WIFI_MESSAGE_BUFFER_SIZE];
    int sock;

    // Initialize Wi-Fi module with FreeRTOS support
    if (cyw43_arch_init())
    {
        printf("Failed to initialize Wi-Fi.\n");
        return;
    }

    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");

    // Attempt to connect to Wi-Fi (SSID and password should be set in environment or code)
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Failed to connect to Wi-Fi.\n");
        return;
    }

    printf("Connected to Wi-Fi.\n");

    while (1)
    {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
        {
            printf("Socket creation failed\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr.s_addr);

        if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            printf("Connection to server failed\n");
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        printf("Connected to server\n");

        while (1)
        {
            size_t received = xMessageBufferReceive(xMotorSpeedMessageBuffer, (void *)message, sizeof(message), portMAX_DELAY);
            if (received > 0)
            {
                int bytes_sent = send(sock, message, received, 0);
                if (bytes_sent < 0)
                {
                    printf("Failed to send message\n");
                    close(sock);
                    break;
                }
                printf("Message sent: %s\n", message);
            }
        }
        close(sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// MARK: - Motor control task
// Motor control task
void motor_task(__unused void *params)
{
    char speed_message[WIFI_MESSAGE_BUFFER_SIZE];
    float timer_value = 250000;

    while (1)
    {
        // Read motor speed
        // left_speed = getLeftWheelSpeed(timer_value);
        // right_speed = getRightWheelSpeed(timer_value);
        // Format motor speed data
        snprintf(speed_message, sizeof(speed_message), "Left: %.5f cm/s| Right: %.5f cm/s|", left_speed, right_speed);

        // // Clear any extra data in buffer before sending
        // memset(speed_message + strlen(speed_message), '\0', sizeof(speed_message) - strlen(speed_message));

        // Send speed data to the Wi-Fi task
        xMessageBufferSend(xMotorSpeedMessageBuffer, (void *)speed_message, strlen(speed_message), 0);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// MARK: - Ultrasonic sensor task
// Ultrasonic sensor task
void ultrasonic_task(__unused void *params)
{
    while (1)
    {
        // Read ultrasonic sensor data
        // double distance = getDistance(TRIG_PIN, ECHO_PIN);
        // printf("Distance: %.2f cm\n", distance);

        sentTrigPulse(TRIG_PIN, ECHO_PIN);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vLaunch(void)
{
    xMotorSpeedMessageBuffer = xMessageBufferCreate(WIFI_MESSAGE_BUFFER_SIZE);

    if (!xMotorSpeedMessageBuffer)
    {
        printf("Failed to create message buffer.\n");
        return;
    }

    TaskHandle_t wifiTaskHandle;
    TaskHandle_t motorTaskHandle;
    TaskHandle_t ultrasonicTaskHandle;

    xTaskCreate(wifi_task, "WifiTask", 4096, NULL, WIFI_TASK_PRIORITY, &wifiTaskHandle);
    xTaskCreate(motor_task, "MotorTask", 4096, NULL, MOTOR_TASK_PRIORITY, &motorTaskHandle);
    xTaskCreate(ultrasonic_task, "UltrasonicTask", 4096, NULL, MOTOR_TASK_PRIORITY, &ultrasonicTaskHandle);

    vTaskStartScheduler();
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
    gpio_set_irq_enabled_with_callback(RIGHT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(LEFT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true);

    // Initialize PID controllers for both wheels with appropriate values
    // MARK: PID
    // setup_pid(&left_pid, 0.5, 0, 0.0);  // Adjust Kp, Ki, Kd as needed for left motor
    // setup_pid(&right_pid, 0.7, 0, 0.0); // Adjust Kp, Ki, Kd as needed for right motor
    setup_pid(&left_pid, 0.40, 0.01, 0); // Adjust Kp, Ki, Kd as needed for left motor 0.017 0.015
    setup_pid(&right_pid, 0.7, 0.05, 0); // Adjust Kp, Ki, Kd as needed for right motor 0.002 0.0027 .65->P)

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
            ramp_up_duty_cycle(initial_duty_cycle, target_duty_cycle, LEFT_PWM_PIN, RIGHT_PWM_PIN);

            // Now motors have reached target speed, and PID control can take over
            vLaunch();
        }

        // sleep_ms(100);
    }

    return 0;
}