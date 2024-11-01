#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/sockets.h"

#define PORT 65439
#define BUFFER_SIZE 1024
#define MAX_PENDING_CONNECTIONS 3

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *server_task) // Function to check if stack overflow occurs
{
    printf("Stack overflow in task: %s\n", server_task);
    while (1)
        ; // Trap the error
}

void handle_client(int client_socket)
{
    char buffer[BUFFER_SIZE];
    int bytes_received;

    while ((bytes_received = recv(client_socket, buffer, BUFFER_SIZE - 1, 0)) > 0)
    {
        buffer[bytes_received] = '\0'; // Null-terminate received data

        char *message = strtok(buffer, "|");
        while (message != NULL)
        {
            printf("Message received from client: %s\n", message);
            message = strtok(NULL, "|");
        }
    }

    if (bytes_received < 0)
    {
        perror("Error receiving data");
    }

    close(client_socket);
    printf("Client disconnected.\n");
}

void server_task(__unused void *params)
{
    sleep_ms(5000);

    if (cyw43_arch_init())
    {
        printf("Failed to initialize Wi-Fi.\n");
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

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
    {
        printf("Failed to create socket.\n");
        return;
    }
    printf("Socket Created.\n");

    int optval = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(PORT)};

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        printf("Failed to bind socket.\n");
        close(server_fd);
        return;
    }
    printf("Socket Bound.\n");

    if (listen(server_fd, MAX_PENDING_CONNECTIONS) < 0)
    {
        printf("Failed to listen on socket.\n");
        close(server_fd);
        return;
    }

    printf("Server listening on port %d\n", PORT);

    while (1)
    {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_addr_len);
        if (client_fd < 0)
        {
            printf("Failed to accept connection.\n");
            continue;
        }

        printf("Client connected.\n");
        handle_client(client_fd);
    }

    close(server_fd);
    printf("Server Socket closed.\n");
}

void vLaunch(void)
{
    TaskHandle_t serverTaskHandle;
    xTaskCreate(server_task, "ServerTask", 8192, NULL, tskIDLE_PRIORITY + 1, &serverTaskHandle);
    vTaskStartScheduler();
}

int main(void)
{
    stdio_init_all();
    sleep_ms(10000);

    printf("Starting server on FreeRTOS\n");
    vLaunch();
    return 0;
}
