#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/sockets.h"

// Define server settings
#define PORT 65439
#define BUFFER_SIZE 2048
#define MAX_PENDING_CONNECTIONS 3

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define configMINIMAL_STACK_SIZE (2048)
#define configCHECK_FOR_STACK_OVERFLOW 2 // Enable stack overflow checking

#define SERVER_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define SERVER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 10240) // Increased stack size for FreeRTOS task

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *server_task) // Function to check if stack overflow occurs
{
    printf("Stack overflow in task: %s\n", server_task);
    while (1)
        ; // Trap the error
}

typedef struct
{
    int direction;
    float left_speed;
    float right_speed;
    float left_distance;
    float right_distance;
} TelemetryData;

void log_telemetry(TelemetryData *data)
{
    printf("Parsed Telemetry Data:\n");
    printf("  Direction: %d\n", data->direction);
    printf("  Left Speed: %.2f cm/s\n", data->left_speed);
    printf("  Right Speed: %.2f cm/s\n", data->right_speed);
    printf("  Left Distance: %.2f cm\n", data->left_distance);
    printf("  Right Distance: %.2f cm\n", data->right_distance);
}
// Print each client message as a new line to match client-side print
void handle_client(int client_socket)
{
    char buffer[BUFFER_SIZE];
    int bytes_received;

    while ((bytes_received = recv(client_socket, buffer, BUFFER_SIZE - 1, 0)) > 0)
    {
        buffer[bytes_received] = '\0'; // Null-terminate the received data
        printf("Raw telemetry data received: %s\n", buffer);

        TelemetryData telemetry;
        if (sscanf(buffer, "%d,%f,%f,%f,%f",
                   &telemetry.direction,
                   &telemetry.left_speed,
                   &telemetry.right_speed,
                   &telemetry.left_distance,
                   &telemetry.right_distance) == 5)
        {
            log_telemetry(&telemetry);
        }
        else
        {
            printf("Error: Unable to parse telemetry data: %s\n", buffer);
        }
    }

    if (bytes_received < 0)
    {
        perror("Error receiving data");
    }

    printf("Client disconnected.\n");
}

void server_task(__unused void *params)
{
    printf("Telemetry Server task starting...\n");

    sleep_ms(5000);

    // Initialize Wi-Fi with FreeRTOS support
    if (cyw43_arch_init())
    {
        printf("Failed to initialize Wi-Fi with FreeRTOS.\n");
        return;
    }

    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 40000))
    {
        printf("Failed to connect to Wi-Fi.\n");

        return;
    }
    printf("Connected to Wi-Fi.\n");

    printf("Server IP: %s\n", ip4addr_ntoa(&cyw43_state.netif[0].ip_addr));

    // Create server socket
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
    {
        printf("Failed to create socket.\n");

        return;
    }
    printf("Socket Created.\n");

    int optval = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    int flag = 1;
    setsockopt(server_fd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(PORT)};

    // Bind socket to address and port
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        printf("Failed to bind socket.\n");
        close(server_fd);

        return;
    }
    printf("Socket Bound.\n");

    // Start listening for incoming connections
    if (listen(server_fd, MAX_PENDING_CONNECTIONS) < 0)
    {
        printf("Failed to listen on socket.\n");
        close(server_fd);

        return;
    }

    printf("Server listening on port %d\n", PORT);

    vTaskDelay(pdMS_TO_TICKS(1000));
    while (1)
    {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);

        // Accept a new connection
        int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_addr_len);
        if (client_fd < 0)
        {
            printf("Failed to accept connection.\n");
            continue;
        }

        printf("Client connected.\n");

        handle_client(client_fd);

        // Close the client socket
        close(client_fd);
        printf("Client Socket closed.\n");
    }

    // Close the server socket and clean up
    close(server_fd);
    printf("Server Socket closed.\n");
}

void vLaunch(void)
{
    TaskHandle_t serverTaskHandle;

    xTaskCreate(server_task, "ServerTask", SERVER_TASK_STACK_SIZE, NULL, SERVER_TASK_PRIORITY + 1, &serverTaskHandle);

    vTaskStartScheduler();
}

int main(void)
{
    stdio_init_all();
    sleep_ms(10000); // Allow time for serial monitor to connect

    printf("Starting server on FreeRTOS\n");
    vLaunch(); // Launch the server task
    return 0;
}
