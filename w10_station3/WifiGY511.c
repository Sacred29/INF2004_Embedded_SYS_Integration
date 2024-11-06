/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/ip4_addr.h" // Maybe need (need to test)
#include "lwip/sockets.h"  // lwIP socket API for networking

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

// Gesture
#include "hardware/i2c.h"
#define I2C_PORT i2c0

// Addresses for data reading
#define ACCEL_ADDR 0x19
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define CTRL_REG1_A 0x20

// Direction and speed mapping thresholds
#define MAX_TILT 16000    // Maximum tilt value for full speed
#define MIN_TILT 2000     // Minimum tilt value to start moving (any tilt below this is ignored)
#define SMA_WINDOW_SIZE 5 // Size of the moving average window

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define configMINIMAL_STACK_SIZE (2048)

#define configCHECK_FOR_STACK_OVERFLOW 2 // Enable stack overflow checking

#define SERVER_IP "192.168.1.81" // Server IP address
#define SERVER_PORT 65432        // Port no. the server is listening on

#define WIFI_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define WIFI_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 2048)
#define SENSOR_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define SENSOR_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 2048)
// Message buffer
MessageBufferHandle_t xControlWifiMessageBuffer;
#define WIFI_MESSAGE_BUFFER_SIZE (60)

// Gesture
// Moving average buffers
int16_t x_buffer[SMA_WINDOW_SIZE] = {0};
int16_t y_buffer[SMA_WINDOW_SIZE] = {0};
int gesture_index = 0;

// Gesture Control Portion START
// Initialize I2C and accelerometer pins
void i2c_init_pins()
{
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
}

// Write to register
void write_register(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buf[] = {reg, data};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}

// Read from register
uint8_t read_register(uint8_t addr, uint8_t reg)
{
    uint8_t data;
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, &data, 1, false);
    return data;
}

// Read 16-bit axis data
int16_t read_axis(uint8_t reg_l, uint8_t reg_h)
{
    uint8_t lsb = read_register(ACCEL_ADDR, reg_l);
    uint8_t msb = read_register(ACCEL_ADDR, reg_h);
    return (int16_t)((msb << 8) | lsb);
}

// Simple Moving Average (SMA) filter
int16_t apply_sma_filter(int16_t *buffer, int16_t new_value)
{
    buffer[gesture_index] = new_value;
    int32_t sum = 0;
    for (int i = 0; i < SMA_WINDOW_SIZE; i++)
    {
        sum += buffer[i];
    }
    return (int16_t)(sum / SMA_WINDOW_SIZE);
}

// map the tilt level as speed
int map_tilt_to_speed(int16_t tilt_value)
{
    int tilt_magnitude = abs(tilt_value);

    if (tilt_magnitude < MIN_TILT)
    {
        return 0; // Below the threshold, no movement
    }

    int speed_percent = (tilt_magnitude - MIN_TILT) * 100 / (MAX_TILT - MIN_TILT);

    // Clip to 100% maximum if value is very close
    if (speed_percent > 100)
    {
        speed_percent = 100;
    }
    return speed_percent;
}

// Determine and print tilt-based speed percentage
void send_tilt_speed(int16_t filtered_x, int16_t filtered_y)
{
    int forward_speed = map_tilt_to_speed(filtered_y);
    int sideways_speed = map_tilt_to_speed(filtered_x);
    char message[50];

    if (filtered_y < -MIN_TILT)
    {
        snprintf(message, sizeof(message), "Forward at %d%% Speed|", forward_speed);
        xMessageBufferSend(xControlWifiMessageBuffer, (void *)message, strlen(message), 0);
    }
    else if (filtered_y > MIN_TILT)
    {
        snprintf(message, sizeof(message), "Backward at %d%% Speed|", forward_speed);
        xMessageBufferSend(xControlWifiMessageBuffer, (void *)message, strlen(message), 0);
    }

    if (filtered_x < -MIN_TILT)
    {
        snprintf(message, sizeof(message), "Right at %d%% Speed|", sideways_speed);
        xMessageBufferSend(xControlWifiMessageBuffer, (void *)message, strlen(message), 0);
    }
    else if (filtered_x > MIN_TILT)
    {
        snprintf(message, sizeof(message), "Left at %d%% Speed|", sideways_speed);
        xMessageBufferSend(xControlWifiMessageBuffer, (void *)message, strlen(message), 0);
    }

    if (forward_speed == 0 && sideways_speed == 0)
    {
        strcpy(message, "Stop|");
        xMessageBufferSend(xControlWifiMessageBuffer, (void *)message, strlen(message), 0);
    }
}

// Wifi Portion START
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *wifi_task) // Function to check if stack overflow occurs
{
    printf("Stack overflow in task: %s\n", wifi_task);
    while (1)
        ; // Trap the error
}

// Wifi Transmission Task
void wifi_task(__unused void *params)
{
    sleep_ms(5000); // Give me time to open serial monitor

    if (cyw43_arch_init()) // Init wifi driver
    {
        printf("Failed to initialize Wi-Fi\n");
        return;
    }
    cyw43_arch_enable_sta_mode(); // Enable wifi station mode. basically tells pico's wifi chip to act as a wifi client and look for available wifi networks to connect to. once connected, router assigns ip address to the pico
    printf("Connecting to Wi-Fi... (On client)\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) // Wifi SSID, PW from CMakeLists.txt (where they are called from the env), authentication type and timeout length
    {
        printf("Failed to connect to Wi-Fi.\n");
        exit(1); // Exit if fail to connect to wifi
    }

    else
    {
        printf("Connected to Wi-Fi.\n");
    }

    // Create client socket. IPv4 (AF_INET), TCP (SOCK_STREAM)
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        printf("Failed to create socket\n");
        return;
    }

    printf("Socket created successfully\n");

    int flag = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

    struct sockaddr_in server_addr;            // Server address struct
    server_addr.sin_family = AF_INET;          // IPv4
    server_addr.sin_port = htons(SERVER_PORT); // Convert port number to network byte order

    if (inet_aton(SERVER_IP, &server_addr.sin_addr) == 0) // inet_aton function converts Internet host address cp from the IPv4 numbers-and-dots notation into binary form, returns nonzero if the address is valid
    {
        printf("Invalid server IP address: %s\n", SERVER_IP);
        close(sock);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));

    // Connect to PC (Server)
    printf("Connecting to server...\n");

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) // connect() params: int socket, const struct sockaddr *address, socklen_t address_len. successful connection returns 0, else -1 and error no.
    {
        printf("Failed to connect to server. errno: %d\n", errno); // Print error no. on failed connection
        close(sock);
        return;
    }

    printf("Connected to server successfully\n");

    // Send a message to the server
    char message[50];

    while (true)
    {
        size_t xReceivedBytes;
        xReceivedBytes = xMessageBufferReceive(xControlWifiMessageBuffer, (void *)message, sizeof(message), portMAX_DELAY);
        if (xReceivedBytes > 0)
        {
            // Null-terminate the received message
            message[xReceivedBytes] = '\0';

            // Send message through socket
            int bytes_sent = send(sock, message, strlen(message), 0); // params: int socket, const void *buffer, size_t length, int flags. Returns no. of bytes sent if successful, else -1 and error no.
            if (bytes_sent < 0)
            {
                printf("Failed to send message\n");
                close(sock);
                return;
            }
            printf("Message sent to server: %s\n", message);
        }
        // vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Close the socket after communication
    close(sock);
    printf("Socket closed.\n");
}

// Gesture Control Task
void sensor_task(__unused void *params)
{
    i2c_init_pins();

    // Initialize accelerometer with 100Hz data rate, enable X, Y, Z axes (refer to docs 0x57)
    write_register(ACCEL_ADDR, CTRL_REG1_A, 0x57);

    while (true)
    {
        // Read raw accelerometer data
        int16_t accel_x = read_axis(OUT_X_L_A, OUT_X_H_A);
        int16_t accel_y = read_axis(OUT_Y_L_A, OUT_Y_H_A);

        // Apply SMA filtering
        int16_t filtered_x = apply_sma_filter(x_buffer, accel_x);
        int16_t filtered_y = apply_sma_filter(y_buffer, accel_y);

        // Move index for circular buffer
        gesture_index = (gesture_index + 1) % SMA_WINDOW_SIZE;

        // Determine tilt direction
        send_tilt_speed(filtered_x, filtered_y);

        sleep_ms(100); // Small delay
    }

    return;
}

void vLaunch(void)
{
    // Create Message Buffer
    xControlWifiMessageBuffer = xMessageBufferCreate(WIFI_MESSAGE_BUFFER_SIZE);

    if (!xControlWifiMessageBuffer)
    {
        printf("Failed to create message buffer.\n");
        return;
    }

    // Create Task Handles
    TaskHandle_t wifiTaskHandle;
    TaskHandle_t sensorTaskHandle;

    // Create Tasks
    xTaskCreate(wifi_task, "WifiTask", WIFI_TASK_STACK_SIZE, NULL, WIFI_TASK_PRIORITY, &wifiTaskHandle);
    xTaskCreate(sensor_task, "SensorTask", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, &sensorTaskHandle);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUM_CORES > 1
    // We must bind the main task to one core (only in NO_SYS mode)
    vTaskCoreAffinitySet(task, 1);
#endif

    vTaskStartScheduler(); // Start FreeRTOS tasks
}

int main(void)
{
    stdio_init_all();
    sleep_ms(10000); // Give time for serial to stabilize

    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;
    rtos_name = "FreeRTOS";
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch(); // Launch main task

    while (true)
    {
    }
    return 0;
}
