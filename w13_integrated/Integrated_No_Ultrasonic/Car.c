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
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h" // Maybe need (need to test)
#include "lwip/sockets.h"  // lwIP socket API for networking
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

// IR
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <string.h>

// Define GPIO pins Right Motor
#define RIGHT_PWM_PIN 3    // GP2 for PWM
#define RIGHT_DIR_PIN1 0   // GP0 for direction
#define RIGHT_DIR_PIN2 1   // GP1 for direction
#define RIGHT_SENSOR_PIN 4 // GPIO for the right IR sensor

// Define GPIO pins Left Motor
#define LEFT_PWM_PIN 11    // GP8 for PWM
#define LEFT_DIR_PIN1 6    // GP6 for direction
#define LEFT_DIR_PIN2 7    // GP7 for direction
#define LEFT_SENSOR_PIN 26 // GPIO for the left IR sensor

// Define GPIO pins Left Motor
#define TRIG_PIN 8 // GP8 for Trigger Pin
#define ECHO_PIN 9 // GP9 for Echo Pin

// Control constants
#define BASELINE_DC 0.4   // 0.65
#define TARGET_SPEED 25.0 // 20.0
#define ADJUSTMENT_FACTOR 0.02
#define PULSES_PER_REVOLUTION 20 // Define the constant with an appropriate value

// LINE FOLLOWING CONTROL CONSTANTS
#define CORRECTION_DC_RIGHT 0.25
#define CORRECTION_DC_LEFT 0
#define WHITE_TIMEOUT 4000

// FREERTOS
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define configMINIMAL_STACK_SIZE (2048)
#define configCHECK_FOR_STACK_OVERFLOW 2 // Enable stack overflow checking
// #define SERVER_IP "172.20.10.7"          // PC/Server IP address
// #define SERVER_PORT 65439                // Port no. the server is listening on
#define RECEIVE_PORT 65431
#define SEND_IP "172.20.10.5"
#define SEND_PORT 65439
#define MY_IP "172.20.10.8"

#define WIFI_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define WIFI_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 10240) // Increased stack size for FreeRTOS task
#define ULTRASONIC_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define ULTRASONIC_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 1024) // Increased stack size for FreeRTOS task
#define IR_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define IR_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 1024) // Increased stack size for FreeRTOS task

// Message buffer
MessageBufferHandle_t xTelemetryDataMessageBuffer;
MessageBufferHandle_t xIRMessageBuffer;
#define WIFI_MESSAGE_BUFFER_SIZE (1024)
#define BUFFER_SIZE 1024
#define MAX_PENDING_CONNECTIONS 6

// Define IR
#define BARCODE_BUFFER 10
#define BTN_PIN 21
#define BTN_PIN_INCREASE 22

volatile bool IR_SWAP = false; // Bool for changing modes
volatile uint16_t adc0_data = 0;
volatile uint16_t adc1_data = 0;
volatile bool data_ready = false;
static uint16_t max_thresh = 0;
static uint16_t min_thresh = 4095;
static uint16_t contrast_thresh = 2000;

typedef struct
{
    char character;
    bool pattern[9];
} BarcodeCharacter;

// Define character-pattern pairs
const BarcodeCharacter barcode_characters[] = {
    {'A', {true, false, false, true, false, false, false, false, true}},
    {'F', {false, false, false, true, true, false, true, false, false}},
    {'Z', {false, false, false, false, true, false, true, true, false}},
    {'*', {false, false, true, false, true, false, false, true, false}},

    {'0', {false, false, true, false, true, true, false, false, false}},
    {'2', {true, false, false, false, false, true, true, false, false}},
    {'4', {true, false, false, false, true, true, false, false, false}},
    {'6', {false, false, false, false, true, true, true, false, false}},
    {'8', {false, false, true, false, false, true, false, false, true}},

    {'C', {false, false, false, true, false, false, true, false, true}},

    {'E', {false, false, false, true, true, false, false, false, true}},
    {'G', {true, false, true, true, false, false, false, false, false}},
    {'I', {false, false, true, true, false, false, true, false, false}},
    {'K', {true, true, false, false, false, false, false, false, true}},
    {'M', {false, true, false, false, false, false, true, false, true}},
    {'O', {false, true, false, false, true, false, false, false, true}},
    {'Q', {true, true, true, false, false, false, false, false, false}},
    {'S', {false, true, true, false, false, false, true, false, false}},
    {'U', {true, false, false, false, false, false, false, true, true}},
    {'W', {false, false, false, false, false, false, true, true, true}},

    {'Y', {false, false, false, false, true, false, false, true, true}},
    {'-', {true, false, true, false, false, false, false, true, false}},
    {' ', {false, false, true, false, false, false, true, true, false}},
    {'/', {false, true, false, false, false, true, false, true, false}},
    {'%', {false, true, false, true, false, true, false, false, false}},

    {'1', {true, false, false, false, false, true, false, false, true}},
    {'3', {false, false, false, false, false, true, true, false, true}},
    {'5', {false, false, false, false, true, true, false, false, true}},
    {'7', {true, false, true, false, false, true, false, false, false}},
    {'9', {false, false, true, false, false, true, true, false, false}},
    {'B', {true, false, false, true, false, false, true, false, false}},
    {'D', {true, false, false, true, true, false, false, false, false}},

    {'H', {false, false, true, true, false, false, false, false, true}},
    {'J', {false, false, true, true, true, false, false, false, false}},
    {'L', {true, true, false, false, false, false, true, false, false}},
    {'N', {true, true, false, false, true, false, false, false, false}},
    {'P', {false, true, false, false, true, false, true, false, false}},
    {'R', {false, true, true, false, false, false, false, false, true}},
    {'T', {false, true, true, false, true, false, false, false, false}},
    {'V', {true, false, false, false, false, false, true, true, false}},
    {'X', {true, false, false, false, true, false, false, true, false}},
    {'Z', {false, false, false, false, true, false, true, true, false}},
    {'.', {false, false, true, false, false, false, false, true, true}},
    {'$', {false, false, false, true, false, true, false, true, false}},
    {'+', {false, true, false, true, false, false, false, true, false}},
};

// IR Prototypes
// void line_follower(uint16_t adc_value, bool isMoving, float baseline_dc);
void line_follower(uint16_t adc_value);
void barcode_detector(uint16_t adc_value);
void classify_bar(uint16_t duration_ms, bool is_black, uint16_t adc_reading);
void barcode_start_check(uint8_t index, bool *isBlack, bool *isWide, uint8_t *start_index, uint8_t *bar_count, bool *started_reading, bool *is_forward, char *message);
void barcode_read_char(bool *isWide, uint8_t *start_index, uint8_t *bar_count, bool *started_reading, bool *is_forward, char *message);
bool adc_timer_callback(repeating_timer_t *rt);
void init_adc_timer();
void process_adc_data();
void add_char_to_string(char *message, char new_char);
// void swap_modes();

// Telemetry Struct
typedef struct
{
    int direction;
    float left_speed;
    float right_speed;
    float left_distance;
    float right_distance;
    double ultrasonic_distance;
} TelemetryData;

// Initialize button pin
const uint BTN_PIN_START = 21;
const uint BTN_PIN_LEFT = 20;
const uint BTN_PIN_RIGHT = 22;

float left_speed = 0.0;
float right_speed = 0.0;

float left_distance = 0.0;
float right_distance = 0.0;

// float timer_value = 500000;
// float timer_value = 100000;
float timer_value = 250000;

absolute_time_t start_time, end_time, turn_start_time, pause_start_time;
bool isDetected = false;

// Define initial duty cycle, target duty cycle, and increment step
const float initial_duty_cycle = 0.5; // Starting point, lower than baseline
const float target_duty_cycle = 0.7;  // Baseline duty cycle
const float ramp_increment = 0.05;    // Increment step for ramp-up
const uint ramp_delay_ms = 250;       // Delay between increments in milliseconds

// PID Controller
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
} PIDController;

// Global PID controllers and moving state
PIDController right_pid, left_pid;
bool isMoving = false; // Tracks movement state

// Movement states using defines
#define STATE_STOP 0
#define STATE_FORWARD 1
#define STATE_REVERSE 2
#define STATE_TURN_LEFT 3
#define STATE_TURN_RIGHT 4

// Instead of MovementState, use int
int currentState = STATE_STOP;

void ramp_up_duty_cycle(float initial_duty_cycle, float target_duty_cycle, uint gpio_pin_left, uint gpio_pin_right);
void set_movement_state(int new_state);
void gpio_callback(uint gpio, uint32_t events);
void setup_gpio_pins();
void setup_pwm(uint gpio, float freq, float duty_cycle);
void set_speed(float duty_cycle, uint gpio_pin);

// // PID Controller
// typedef struct
// {
//     float Kp;
//     float Ki;
//     float Kd;
//     float integral;
//     float prev_error;
// } PIDController;

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

// MARK: Gesture Controls
void set_motor_direction(bool right_forward, bool left_forward)
{
    gpio_put(RIGHT_DIR_PIN1, right_forward ? 0 : 1);
    gpio_put(RIGHT_DIR_PIN2, right_forward ? 1 : 0);
    gpio_put(LEFT_DIR_PIN1, left_forward ? 1 : 0);
    gpio_put(LEFT_DIR_PIN2, left_forward ? 0 : 1);
}

void set_movement_state(int new_state)
{
    currentState = new_state;

    switch (new_state)
    {
    case STATE_FORWARD:
        printf("Moving Forward\n");
        set_motor_direction(true, true);
        break;
    case STATE_REVERSE:
        printf("Moving Backward\n");
        set_motor_direction(false, false);
        break;
    case STATE_TURN_LEFT:
        printf("Turning Left\n");
        set_motor_direction(true, false);
        break;
    case STATE_TURN_RIGHT:
        printf("Turning Right\n");
        set_motor_direction(false, true);
        break;
    case STATE_STOP:
        printf("Stopping\n");
        set_speed(0, RIGHT_PWM_PIN);
        set_speed(0, LEFT_PWM_PIN);
        break;
    }
}

// MARK: Repeating Timer Callback
bool repeating_timer_callback(__unused struct repeating_timer *t)
{
    // sentTrigPulse(TRIG_PIN, ECHO_PIN);
    if (currentState != STATE_STOP)
    {
        // Read wheel speeds
        getLeftWheelInfo(&left_speed, &left_distance);
        getRightWheelInfo(&right_speed, &right_distance);

        printf("Left Speed: %.2f cm/s | Right Speed: %.2f cm/s\n", left_speed, right_speed);

        // Get ultrasonic readings
        double ultrasonic_distance = getDistance(TRIG_PIN, ECHO_PIN);

        TelemetryData telemetry = {
            .direction = currentState,
            .left_speed = left_speed,
            .right_speed = right_speed,
            .left_distance = left_distance,
            .right_distance = right_distance,
            .ultrasonic_distance = ultrasonic_distance};

        // // Serialize telemetry data to a string format
        char telemetry_message[WIFI_MESSAGE_BUFFER_SIZE];
        snprintf(telemetry_message, sizeof(telemetry_message),
                 "%d,%.2f,%.2f,%.2f,%.2f,%.2f",
                 telemetry.direction,
                 telemetry.left_speed,
                 telemetry.right_speed,
                 telemetry.left_distance,
                 telemetry.right_distance,
                 telemetry.ultrasonic_distance);

        xMessageBufferSend(xTelemetryDataMessageBuffer, telemetry_message, strlen(telemetry_message), 0);

        // Compute PID control signals for each wheel
        // The PID controller adjusts the motor speed to maintain the target speed
        float right_control_signal = compute_pid(&right_pid, TARGET_SPEED, right_speed);
        float left_control_signal = compute_pid(&left_pid, TARGET_SPEED, left_speed);

        // Calculate and apply new duty cycles
        float right_duty_cycle = BASELINE_DC + (right_control_signal * ADJUSTMENT_FACTOR);
        float left_duty_cycle = BASELINE_DC + (left_control_signal * ADJUSTMENT_FACTOR);

        // Constrain duty cycles
        right_duty_cycle = right_duty_cycle > 1.0f ? 1.0f : (right_duty_cycle < 0.0f ? 0.0f : right_duty_cycle);
        left_duty_cycle = left_duty_cycle > 1.0f ? 1.0f : (left_duty_cycle < 0.0f ? 0.0f : left_duty_cycle);

        printf("Right Duty Cycle: %.2f | Left Duty Cycle: %.2f\n", right_duty_cycle, left_duty_cycle);

        set_speed(right_duty_cycle, RIGHT_PWM_PIN);
        set_speed(left_duty_cycle, LEFT_PWM_PIN);
    }

    return true; // Keep the timer running
}

// GPIO and PWM setup functions
void setup_gpio_pins()
{
    gpio_init(BTN_PIN_START);
    gpio_set_dir(BTN_PIN_START, GPIO_IN);
    gpio_pull_up(BTN_PIN_START);

    gpio_init(BTN_PIN_LEFT);
    gpio_set_dir(BTN_PIN_LEFT, GPIO_IN);
    gpio_pull_up(BTN_PIN_LEFT);

    gpio_init(BTN_PIN_RIGHT);
    gpio_set_dir(BTN_PIN_RIGHT, GPIO_IN);
    gpio_pull_up(BTN_PIN_RIGHT);

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
    if (gpio == ECHO_PIN) // Avoid repeated detections during turn or pause
    {
        double object_distance = getDistance(TRIG_PIN, ECHO_PIN);
        // printf("Distance: %.2f cm\n", object_distance);
        if (object_distance <= 12 && object_distance != -1 && !isDetected)
        {

            printf("Obstacle detected\n");
            isDetected = true;
            // Stop the motors
            set_movement_state(STATE_STOP);
        }
        else
        {
            isDetected = false;
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

// MARK: - Ultrasonic
void ultrasonic_task(__unused void *params)
{
    while (1)
    {
        // Read ultrasonic sensor data
        // double distance = getDistance(TRIG_PIN, ECHO_PIN);
        // printf("Distance: %.2f cm\n", distance);

        // printf("Sent Trig Pulse\n");
        sentTrigPulse(TRIG_PIN, ECHO_PIN);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// MARK: Motor
void handle_motor(int client_socket)
{

    if (IR_SWAP)
    {
        // Ignore remote commands when in line following mode
        // printf("Line-following mode active. Ignoring remote commands.\n");
        return;
    }
    int received_number;
    char buffer[BUFFER_SIZE];

    // Attempt to receive data
    int bytes_received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
    if (bytes_received > 0)
    {
        buffer[bytes_received] = '\0'; // Null-terminate the received data
        printf("Received message: %s\n", buffer);

        // Convert the received message to an integer and map it to movement
        received_number = atoi(buffer); // Convert buffer to integer

        switch (received_number)
        {
        case 0:
            set_movement_state(STATE_STOP);
            break;
        case 1:
            set_movement_state(STATE_FORWARD);
            break;
        case 2:
            set_movement_state(STATE_REVERSE);
            break;
        case 3:
            set_movement_state(STATE_TURN_LEFT);
            break;
        case 4:
            set_movement_state(STATE_TURN_RIGHT);
            break;
        default:
            printf("Invalid command received\n");
            break;
        }
    }
    else if (bytes_received < 0)
    {
        // Handle non-blocking mode where no data is available
        if (errno == EAGAIN || errno == EWOULDBLOCK) // If recv returns -1 with errno set to EAGAIN or EWOULDBLOCK, it means there is no data available at the moment, so the function returns without processing.
        {
            // No data available right now, return and try again later
            return;
        }
        else
        {
            // An actual error occurred
            perror("Error receiving data");
        }
    }
    else // bytes_received == 0 indicates client closed the connection
    {
        printf("Client disconnected.\n");
        close(client_socket); // Clean up the socket
    }
}

// MARK: Wifi Send Receive Task
void wifi_send_receive_task(__unused void *params)
{
    sleep_ms(5000); // Wait for serial monitor setup

    printf("Initializing Wi-Fi stack...\n");

    if (cyw43_arch_init())
    {
        printf("Failed to initialize Wi-Fi stack.\n");
        return;
    }
    cyw43_arch_enable_sta_mode(); // Enable station mode once
    printf("Wi-Fi initialized.\n");

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Failed to connect to Wi-Fi.\n");
        return;
    }
    printf("Connected to Wi-Fi. IP Address: %s\n", ip4addr_ntoa(&cyw43_state.netif[0].ip_addr));

    // Set up the remote control server
    int receive_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (receive_sock < 0)
    {
        printf("Failed to create receive socket.\n");
        return;
    }
    printf("Receive socket created successfully.\n");

    // Set receive_sock to non-blocking mode
    int flags = fcntl(receive_sock, F_GETFL, 0);
    if (flags < 0 || fcntl(receive_sock, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        printf("Failed to set receive socket to non-blocking mode.\n");
    }
    struct sockaddr_in receive_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(RECEIVE_PORT)};

    // Bind and listen for incoming connections
    if (bind(receive_sock, (struct sockaddr *)&receive_addr, sizeof(receive_addr)) < 0)
    {
        printf("Failed to bind receive socket.\n");
        close(receive_sock);
        return;
    }

    if (listen(receive_sock, MAX_PENDING_CONNECTIONS) < 0)
    {
        printf("Failed to listen on receive socket.\n");
        close(receive_sock);
        return;
    }
    printf("Listening for incoming connections on port %d (remote control).\n", RECEIVE_PORT);

    // Connect to the telemetry server
    int send_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (send_sock < 0)
    {
        printf("Failed to create send socket.\n");
        close(receive_sock);
        return;
    }
    printf("Send socket created successfully.\n");

    struct sockaddr_in send_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SEND_PORT)};

    if (inet_aton(SEND_IP, &send_addr.sin_addr) == 0)
    {
        printf("Invalid telemetry server IP address: %s\n", SEND_IP);
        close(send_sock);
        close(receive_sock);
        return;
    }

    printf("Connecting to telemetry server...\n");
    if (connect(send_sock, (struct sockaddr *)&send_addr, sizeof(send_addr)) < 0)
    {
        printf("Failed to connect to telemetry server.\n");
        close(send_sock);
        close(receive_sock);
        return;
    }
    printf("Connected to telemetry server successfully.\n");

    // Maintain a list of client sockets
    int client_sockets[MAX_PENDING_CONNECTIONS] = {0}; // Initialize to 0
    fd_set read_fds;
    char telemetry_message[WIFI_MESSAGE_BUFFER_SIZE];
    char ir_message[WIFI_MESSAGE_BUFFER_SIZE];

    while (1)
    {
        // Prepare the file descriptor set
        FD_ZERO(&read_fds);
        FD_SET(receive_sock, &read_fds);
        int max_sd = receive_sock;

        // Add client sockets to the read set
        for (int i = 0; i < MAX_PENDING_CONNECTIONS; i++)
        {
            if (client_sockets[i] > 0)
            {
                FD_SET(client_sockets[i], &read_fds);
                if (client_sockets[i] > max_sd)
                {
                    max_sd = client_sockets[i];
                }
            }
        }

        struct timeval timeout = {.tv_sec = 1, .tv_usec = 0}; // 1-second timeout
        int activity = select(max_sd + 1, &read_fds, NULL, NULL, &timeout);

        if (activity < 0 && errno != EINTR)
        {
            printf("Select error.\n");
            continue;
        }

        // Check for new connections
        if (FD_ISSET(receive_sock, &read_fds))
        {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);

            int client_sock = accept(receive_sock, (struct sockaddr *)&client_addr, &client_len);
            if (client_sock < 0)
            {
                printf("Failed to accept client connection.\n");
                continue;
            }

            printf("New client connected.\n");

            // Set client socket to non-blocking mode
            int flags = fcntl(client_sock, F_GETFL, 0);
            fcntl(client_sock, F_SETFL, flags | O_NONBLOCK);

            // Add the new client socket to the list
            for (int i = 0; i < MAX_PENDING_CONNECTIONS; i++)
            {
                if (client_sockets[i] == 0)
                {
                    client_sockets[i] = client_sock;
                    printf("Added client socket to list\n");
                    break;
                }
            }
        }

        // Check all client sockets for incoming data
        for (int i = 0; i < MAX_PENDING_CONNECTIONS; i++)
        {
            int client_sock = client_sockets[i];
            if (client_sock > 0 && FD_ISSET(client_sock, &read_fds))
            {
                handle_motor(client_sock);

                // Check if the client disconnected
                char peek_buffer[1];
                if (recv(client_sock, peek_buffer, sizeof(peek_buffer), MSG_PEEK) == 0)
                {
                    printf("Client disconnected.\n");
                    close(client_sock);
                    client_sockets[i] = 0; // Remove from the list
                }
            }
        }

        // Send telemetry data
        size_t telemetry_bytes = xMessageBufferReceive(xTelemetryDataMessageBuffer, telemetry_message, sizeof(telemetry_message), 0);
        if (telemetry_bytes > 0)
        {
            telemetry_message[telemetry_bytes] = '\0'; // Null-terminate the string

            // Add the "T:" prefix
            char prefixed_message[WIFI_MESSAGE_BUFFER_SIZE];
            snprintf(prefixed_message, sizeof(prefixed_message), "T:%s", telemetry_message);

            int bytes_sent = send(send_sock, prefixed_message, strlen(prefixed_message), 0);
            if (bytes_sent < 0)
            {
                printf("Failed to send telemetry message.\n");
            }
            else
            {
                printf("Sent telemetry: %s\n", prefixed_message);
            }
        }

        // Send IR data
        size_t ir_bytes = xMessageBufferReceive(xIRMessageBuffer, ir_message, sizeof(ir_message), 0);
        if (ir_bytes > 0)
        {
            ir_message[ir_bytes] = '\0'; // Null-terminate the string

            // Add the "I:" prefix
            char prefixed_message[WIFI_MESSAGE_BUFFER_SIZE];
            snprintf(prefixed_message, sizeof(prefixed_message), "I:%s", ir_message);

            int bytes_sent = send(send_sock, prefixed_message, strlen(prefixed_message), 0);
            if (bytes_sent < 0)
            {
                printf("Failed to send IR message.\n");
            }
            else
            {
                printf("Sent IR data: %s\n", prefixed_message);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay for task scheduling
    }

    // Clean up sockets
    for (int i = 0; i < MAX_PENDING_CONNECTIONS; i++)
    {
        if (client_sockets[i] > 0)
        {
            close(client_sockets[i]);
        }
    }
    close(send_sock);
    close(receive_sock);
    printf("Sockets closed. Task terminated.\n");
}

// MARK: IR sensor task
void ir_task(__unused void *params)
{
    while (true)
    {
        // Handle ADC data
        if (data_ready)
        {
            process_adc_data();

            data_ready = false; // Reset the flag after processing
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for task scheduling
    }
}

void vLaunch(void)
{
    printf("VLaunch!!\n");

    // Telemetry message buffer
    xTelemetryDataMessageBuffer = xMessageBufferCreate(WIFI_MESSAGE_BUFFER_SIZE);

    if (!xTelemetryDataMessageBuffer)
    {
        printf("Failed to create telemetry message buffer.\n");
        return;
    }

    // IR message buffer
    xIRMessageBuffer = xMessageBufferCreate(WIFI_MESSAGE_BUFFER_SIZE);

    if (!xIRMessageBuffer)
    {
        printf("Failed to create IR message buffer.\n");
        return;
    }

    TaskHandle_t wifiSendReceiveTaskHandle;
    TaskHandle_t ultrasonicTaskHandle;
    TaskHandle_t irTaskHandle;

    if (xTaskCreate(ultrasonic_task, "UltrasonicTask", ULTRASONIC_TASK_STACK_SIZE, NULL, ULTRASONIC_TASK_PRIORITY, &ultrasonicTaskHandle) != pdPASS)
    {
        printf("Failed to create Ultrasonic Task\n");
    }

    if (xTaskCreate(ir_task, "IRTask", IR_TASK_STACK_SIZE, NULL, IR_TASK_PRIORITY, &irTaskHandle) != pdPASS)
    {
        printf("Failed to create IR Task\n");
    }

    if (xTaskCreate(wifi_send_receive_task, "WiFiSendReceiveTask", WIFI_TASK_STACK_SIZE, NULL, WIFI_TASK_PRIORITY, &wifiSendReceiveTaskHandle) != pdPASS)
    {
        printf("Failed to create WiFi Send/Receive Task\n");
    }

    printf("VLaunch!! Complete\n");
    vTaskStartScheduler();
}

// MARK: Main
int main()
{
    stdio_init_all();
    sleep_ms(5000);
    printf("hello peepo we r in main\n");

    setup_gpio_pins();
    setupWheelEncoderPins(LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN);
    gpio_set_irq_enabled_with_callback(RIGHT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(LEFT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE, true);

    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);
    gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_RISE, true);
    printf("Ultrasonic setup complete\n");

    // start adc repeating timers
    init_adc_timer();
    printf("ADC timer init complete\n");

    // Initialize PID controllers for both wheels with appropriate values
    // MARK: PID
    setup_pid(&left_pid, 0.14, 0.012, 0.001);
    setup_pid(&right_pid, 0.215, 0.045, 0.002);

    struct repeating_timer timer;
    add_repeating_timer_us(timer_value, repeating_timer_callback, NULL, &timer);

    // Set up PWM on GPIO2 and GPIO8
    setup_pwm(RIGHT_PWM_PIN, 100.0f, 0.0f);
    setup_pwm(LEFT_PWM_PIN, 100.0f, 0.0f);

    vLaunch();

    printf("End of main\n");
    return 0;
}

// MARK: IR functions
void line_follower(uint16_t adc_value)
{

    static uint count_timeout = 0;
    printf("lineFollower\n");
    set_motor_direction(true, true);
    // if (count_timeout <= WHITE_TIMEOUT)
    // {
    if (adc_value >= contrast_thresh)
    {
        // Black detected: Move Forward
        set_speed(BASELINE_DC, RIGHT_PWM_PIN);
        set_speed(BASELINE_DC, LEFT_PWM_PIN);
        count_timeout = 0;
        printf("Black /%u\n", adc0_data);
    }
    else
    {
        // White detected: Correct motor direction
        set_speed(CORRECTION_DC_RIGHT, RIGHT_PWM_PIN);
        set_speed(CORRECTION_DC_LEFT, LEFT_PWM_PIN);
        count_timeout++;
        printf("White: %u\n", adc0_data);
    }
    // }
    // else
    // {
    //     set_speed(0, RIGHT_PWM_PIN);
    //     set_speed(0, LEFT_PWM_PIN);
    //     set_motor_direction(true, false);

    //     static uint i=0;
    //     while(i <= count_timeout) {
    //         set_speed(CORRECTION_DC_RIGHT, RIGHT_PWM_PIN);
    //         set_speed(CORRECTION_DC_LEFT, LEFT_PWM_PIN);
    //         i++;
    //     }

    // }
}

// Timer callback to sample ADCs
bool adc_timer_callback(repeating_timer_t *rt)
{
    // Read ADC0 and ADC1
    adc_select_input(1);
    adc0_data = adc_read();

    adc_select_input(2);
    adc1_data = adc_read();
    // printf("ADC 27: %u\n", adc0_data);
    // printf("ADC 28: %u\n", adc1_data);

    data_ready = true; // Signal that new data is available
    return true;       // Keep repeating
}

// IR Functions

// Function to initialize ADC and Timer
void init_adc_timer()
{
    adc_init();
    adc_gpio_init(27); // Initialize GPIO26 for ADC0
    adc_gpio_init(28); // Initialize GPIO27 for ADC1

    // Set up a repeating timer every 1 ms
    static repeating_timer_t timer;
    add_repeating_timer_ms(-1, adc_timer_callback, NULL, &timer); // polling at 1 ms
}

void barcode_detector(uint16_t adc_value)
{
    static bool last_color_black = false;
    static uint16_t count_time = 0;
    bool current_color_black = (adc_value >= contrast_thresh);
    if (current_color_black != last_color_black)
    {
        // printf("time: %u\n",count_time);
        classify_bar(count_time, last_color_black, adc_value);
        count_time = 0;
        last_color_black = current_color_black;
    }
    else
    {
        count_time++;
    }
}

void classify_bar(uint16_t duration_ms, bool is_black, uint16_t adc_reading)
{
    // printf("Black: %d, Duration %u, reading: %u\n", is_black, duration_ms, adc_reading);

    static uint16_t durations[BARCODE_BUFFER] = {0};
    static bool isBlack[BARCODE_BUFFER] = {0};
    static bool isWide[BARCODE_BUFFER] = {0};
    static uint8_t index = 0;
    static uint8_t count = 0;
    static uint16_t total_duration = 0; // Running total for moving average

    // Update the total duration by removing the old value at index and adding the new value
    total_duration -= durations[index];
    total_duration += duration_ms;

    durations[index] = duration_ms;
    isBlack[index] = is_black;

    if (count < BARCODE_BUFFER)
    {
        count++;
    }

    // Calculate the moving average only when the buffer is full
    if (count == BARCODE_BUFFER)
    {
        // Get the min and max values of durations
        uint16_t width_threshold = (uint16_t)ceil((double)(total_duration / BARCODE_BUFFER) * 1.33);

        uint8_t i = index;

        do
        {
            isWide[i] = (durations[i] > width_threshold);
            i = (i + 1) % BARCODE_BUFFER; // Move to the next element in a circular manner
        } while (i != index); // Loop until we've completed one full pass

        // printf("isWide buffer: ");
        // for (uint8_t i = 0; i < BARCODE_BUFFER; i++)
        // {
        //     printf("%d ", isWide[i]);
        // }
        // printf("\t");
        // barcode check
        static uint8_t start_index;
        static uint8_t bar_count;
        static bool started_reading = false;
        static bool isForward;
        static char message[BARCODE_BUFFER] = {'\0'};
        barcode_start_check(index, isBlack, isWide, &start_index, &bar_count, &started_reading, &isForward, message);

        barcode_read_char(isWide, &start_index, &bar_count, &started_reading, &isForward, message);

        // printf("\tBlack: %d, Duration %u, reading: %u, index: %u width_threshold: %u\n", is_black, duration_ms, adc_reading, index, width_threshold);
    }

    index = (index + 1) % BARCODE_BUFFER;
}

void barcode_start_check(uint8_t index, bool *isBlack, bool *isWide, uint8_t *start_index, uint8_t *bar_count, bool *started_reading, bool *is_forward, char *message)
{
    uint8_t read_index = (index + 1) % BARCODE_BUFFER;
    bool checksum[BARCODE_BUFFER - 1] = {false, false, true, false, true, false, false, true, false};

    if (isBlack[read_index])
    {
        uint8_t count_forward = 0;
        uint8_t count_reverse = 0;
        uint8_t read_index_traverse = read_index;

        // Loop through and check both forward and reverse patterns
        for (uint8_t i = 0; i < BARCODE_BUFFER - 1; ++i)
        {
            // Check forward direction
            if (isWide[read_index_traverse] == checksum[count_forward])
            {
                count_forward++;
            }
            // Check reverse direction
            if (isWide[read_index_traverse] == checksum[(BARCODE_BUFFER - 2) - count_reverse])
            {
                count_reverse++;
            }

            // Advance in circular buffer
            read_index_traverse = (read_index_traverse + 1) % BARCODE_BUFFER;

            // If either pattern matches fully
            if (count_forward == BARCODE_BUFFER - 1 || count_reverse == BARCODE_BUFFER - 1)
            {
                *start_index = read_index;                           // Set start_index
                *bar_count = 0;                                      // Reset bar_count
                *started_reading = !*started_reading;                // Toggle started_reading
                *is_forward = (count_forward == BARCODE_BUFFER - 1); // True if forward, false if reverse
                *message = '\0';
                // printf("START (%s order):\n", *is_forward ? "normal" : "reverse");
                return; // Exit function after successful match
            }

            // If both counts failed, exit early
            if (count_forward != i + 1 && count_reverse != i + 1)
            {
                break;
            }
        }
    }
}

void barcode_read_char(bool *isWide, uint8_t *start_index, uint8_t *bar_count, bool *started_reading, bool *is_forward, char *message)
{

    if (*started_reading)
    {
        // printf("YES\n");
        (*bar_count)++;
        if (*bar_count == BARCODE_BUFFER)
        {
            // printf("HERE!\n");

            // Loop through each defined barcode character
            for (size_t char_idx = 0; char_idx < sizeof(barcode_characters) / sizeof(barcode_characters[0]); ++char_idx)
            {
                const bool *checksum = barcode_characters[char_idx].pattern;
                uint32_t count = 0;
                uint32_t temp_index = *start_index;

                // Compare isWide with checksum based on the direction indicated by is_forward
                while (count < 9)
                {
                    // Forward order: compare isWide[temp_index] with checksum[count]
                    // Reverse order: compare isWide[temp_index] with checksum[8 - count]
                    if (isWide[temp_index] != checksum[*is_forward ? count : (8 - count)])
                    {
                        break;
                    }

                    count++;
                    temp_index = (temp_index + 1) % BARCODE_BUFFER;
                }

                if (count == 9)
                { // All 9 bits matched
                    printf("Detected character: %c\n", barcode_characters[char_idx].character);
                    add_char_to_string(message, barcode_characters[char_idx].character);
                    // xMessageBufferSend(xControlWifiMessageBuffer, (void *)message, strlen(message), 0);
                    // Ensure it stops reading.
                    // Sometimes the start reading function does not catch
                    if (barcode_characters[char_idx].character == '*')
                    {
                        *started_reading = false;
                        message[strlen(message) - 1] = '\0';
                        printf("SEND TO WIFI\n");
                        // send message here
                        printf("Message: %s\n", message);
                        xMessageBufferSend(xIRMessageBuffer, (void *)message, strlen(message), 0);

                        *message = '\0';
                    }

                    // Reset the reading state after a successful read
                    *bar_count = 0;
                    break;
                }
            }
        }
    }
}

// Function to process data from ADC0 and ADC1
// void process_adc_data(bool isMoving, float baseline_dc)
void process_adc_data()
{
    // adc data values are global volatile
    // line_follower(adc0_data, isMoving, baseline_dc);
    if (IR_SWAP)
    {
        // Only run line-follower logic in line-following mode
        line_follower(adc1_data);
        barcode_detector(adc0_data);
    }
    else
    {
        // Check for black line and switch mode
        if (adc0_data >= 3500)
        {
            IR_SWAP = true; // Activate line-following mode

            // const char *swapMessage = "Black line detected. Switching to line-following mode.\n"; // Send the message to the buffer
            // if (xMessageBufferSend(xIRMessageBuffer, (void *)swapMessage, strlen(swapMessage), 0) == 0)
            // {
            //     printf("Failed to send IR message to buffer.\n");
            // }
        }
    }
}

void add_char_to_string(char *message, char new_char)
{
    // Find the current length of the string
    int length = strlen(message);

    // Ensure there is room for the new character and the null terminator
    if (length < BARCODE_BUFFER - 1)
    {
        message[length] = new_char; // Add the new character at the end
        message[length + 1] = '\0'; // Null-terminate the string
    }
}
