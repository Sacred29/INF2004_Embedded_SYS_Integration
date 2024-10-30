/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/ip4_addr.h" // Maybe need (need to test)
#include "lwip/sockets.h"  // lwIP socket API for networking

#include "FreeRTOS.h"
#include "task.h"
#include "ping.h"

#ifndef PING_ADDR
#define PING_ADDR "142.251.35.196"
#endif
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define configMINIMAL_STACK_SIZE (2048)

#define configCHECK_FOR_STACK_OVERFLOW 2 // Enable stack overflow checking

#define SERVER_IP "172.20.10.2" // PC/Server IP address
#define SERVER_PORT 65432       // Port no. the server is listening on

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 10240) // Increased stack size for FreeRTOS task

#define BUFFER_SIZE 1024 // Msg buffer size

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) // Function to check if stack overflow occurs
{
    printf("Stack overflow in task: %s\n", pcTaskName);
    while (1)
        ; // Trap the error
}

void main_task(__unused void *params)
{
    printf("1st print: Free heap size: %u bytes\n", xPortGetFreeHeapSize());
    sleep_ms(5000); // Give me time to open serial monitor

    if (cyw43_arch_init()) // Init wifi driver
    {
        printf("Failed to initialize Wi-Fi\n");
        return;
    }
    cyw43_arch_enable_sta_mode(); // Enable wifi station mode. basically tells pico's wifi chip to act as a wifi client and look for available wifi networks to connect to. once connected, router assigns ip address to the pico
    printf("Connecting to Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) // Wifi SSID, PW from CMakeLists.txt (where they are called from the env), authentication type and timeout length
    {
        printf("Failed to connect to Wi-Fi.\n");
        exit(1); // Exit if fail to connect to wifi
    }

    else
    {
        printf("Connected to Wi-Fi.\n");
    }

    printf("2nd print: Free heap size: %u bytes\n", xPortGetFreeHeapSize());

    printf("Heap size before socket creation: %u bytes\n", xPortGetFreeHeapSize());

    // Create client socket. IPv4 (AF_INET), TCP (SOCK_STREAM)
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        printf("Failed to create socket\n");
        return;
    }

    printf("Socket created successfully\n");
    printf("Heap size after socket creation: %u bytes\n", xPortGetFreeHeapSize());

    struct sockaddr_in server_addr;            // Server address struct
    server_addr.sin_family = AF_INET;          // IPv4
    server_addr.sin_port = htons(SERVER_PORT); // Convert port number to network byte order

    if (inet_aton(SERVER_IP, &server_addr.sin_addr) == 0) // inet_aton function converts Internet host address cp from the IPv4 numbers-and-dots notation into binary form, returns nonzero if the address is valid
    {
        printf("Invalid server IP address\n");
        close(sock);
        return;
    }

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
    const char *message = "Hello from Pico W!";

    int bytes_sent = send(sock, message, strlen(message), 0); // params: int socket, const void *buffer, size_t length, int flags. Returns no. of bytes sent if successful, else -1 and error no.
    if (bytes_sent < 0)
    {
        printf("Failed to send message\n");
        close(sock);
        return;
    }
    printf("Message sent to server: %s\n", message);

    // Receive a message from the server
    char recv_buffer[BUFFER_SIZE];                                            // Define recv buffer
    int bytes_received = recv(sock, recv_buffer, sizeof(recv_buffer) - 1, 0); // recv reads RAW data from a socket, last parameter is "flags" (types of msg transmission). Returns no. of bytes received if successful, else -1 and error no.

    sleep_ms(3000); // Incase server needs time to send msg (maybe can remove)

    if (bytes_received > 0)
    {
        // If message received, print
        recv_buffer[bytes_received] = '\0'; // Null-terminate the received data to print safely, because C strings are null-terminated
        printf("Message received from server: %s\n", recv_buffer);
    }

    else
    {
        printf("Failed to receive message from server.\n");
    }

    // Wait a bit before closing the socket (maybe can remove)
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Close the socket after communication
    close(sock);
    printf("Socket closed.\n");

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Keep the task alive
    }
}

void vLaunch(void)
{
    TaskHandle_t task;
    // Increase stack size for the main task
    xTaskCreate(main_task, "TestMainThread", TASK_STACK_SIZE, NULL, TEST_TASK_PRIORITY, &task);

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
    // #if (portSUPPORT_SMP == 1)
    //     rtos_name = "FreeRTOS SMP";
    // #else
    rtos_name = "FreeRTOS";
    // #endif

    // #if (portSUPPORT_SMP == 1) && (configNUM_CORES == 2)
    //     printf("Starting %s on both cores:\n", rtos_name);
    //     vLaunch();
    // #elif (RUN_FREERTOS_ON_CORE == 1)
    //     printf("Starting %s on core 1:\n", rtos_name);
    //     multicore_launch_core1(vLaunch);
    //     while (true)
    //         ;
    // #else
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch(); // Launch main task
    // #endif
    return 0;
}
