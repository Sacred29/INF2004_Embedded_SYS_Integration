#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h" // Maybe need (need to test)
#include "lwip/sockets.h"  // lwIP socket API for networking
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
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

// Direction mapping thresholds
#define MIN_TILT 5000     // Minimum tilt value to start detecting movement
#define SMA_WINDOW_SIZE 5 // Size of the moving average window

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define configMINIMAL_STACK_SIZE (2048)
#define configCHECK_FOR_STACK_OVERFLOW 2 // Enable stack overflow checking
// #define SERVER_IP "172.20.10.5" // PC/Server IP address
#define CAR_IP "172.20.10.8"
#define CAR_PORT 65431

#define WIFI_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define WIFI_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 2048)
#define SENSOR_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define SENSOR_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 2048)
// Message buffer
MessageBufferHandle_t xControlWifiMessageBuffer;
#define WIFI_MESSAGE_BUFFER_SIZE (60)

// Moving average buffers
int16_t x_buffer[SMA_WINDOW_SIZE] = {0};
int16_t y_buffer[SMA_WINDOW_SIZE] = {0};
int sma_index = 0;
int last_direction = 0;

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
    buffer[sma_index] = new_value;
    int32_t sum = 0;
    for (int i = 0; i < SMA_WINDOW_SIZE; i++)
    {
        sum += buffer[i];
    }
    return (int16_t)(sum / SMA_WINDOW_SIZE);
}

int determine_direction(int16_t filtered_x, int16_t filtered_y)
{

    int current_direction = 0;

    if (filtered_y < -MIN_TILT)
    {
        current_direction = 1; // Forward
    }
    else if (filtered_y > MIN_TILT)
    {
        current_direction = 2; // Backward
    }
    else if (filtered_x < -MIN_TILT)
    {
        current_direction = 4; // Right
    }
    else if (filtered_x > MIN_TILT)
    {
        current_direction = 3; // Left
    }
    else
    {
        current_direction = 0; // Stop
    }

    if (current_direction != last_direction)
    {
        printf("direction is %d\n", current_direction);
        char direction_message[4]; // Sufficient to hold 3 digits and null terminator
        snprintf(direction_message, sizeof(direction_message), "%d", current_direction);
        xMessageBufferSend(xControlWifiMessageBuffer, (void *)direction_message, strlen(direction_message), 0);
    }

    last_direction = current_direction;
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
        sma_index = (sma_index + 1) % SMA_WINDOW_SIZE;

        // Determine tilt direction
        determine_direction(filtered_x, filtered_y);

        sleep_ms(100); // Small delay
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
    printf("Connecting to Wi-Fi... (On GY511)\n");

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

    struct sockaddr_in server_addr;         // Server address struct
    server_addr.sin_family = AF_INET;       // IPv4
    server_addr.sin_port = htons(CAR_PORT); // Convert port number to network byte order

    if (inet_aton(CAR_IP, &server_addr.sin_addr) == 0) // inet_aton function converts Internet host address cp from the IPv4 numbers-and-dots notation into binary form, returns nonzero if the address is valid
    {
        printf("Invalid car IP address: %s\n", CAR_IP);
        close(sock);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));

    // Connect to PC (Server)
    printf("Connecting to car...\n");

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) // connect() params: int socket, const struct sockaddr *address, socklen_t address_len. successful connection returns 0, else -1 and error no.
    {
        printf("Failed to connect to car. errno: %d\n", errno); // Print error no. on failed connection
        close(sock);
        return;
    }

    printf("Connected to car successfully\n");

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
    }
    // Close the socket after communication
    close(sock);
    printf("Socket closed.\n");
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

int main()
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