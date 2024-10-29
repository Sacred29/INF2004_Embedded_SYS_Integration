#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <winsock2.h> // Winsock2 for windows sockets
#include <ws2tcpip.h> // Additional utilities for windows sockets

#pragma comment(lib, "ws2_32.lib") // Link Winsock library

#define PORT 65432 // Port to listen on

#define BUFFER_SIZE 1024 // Size of the buffer for receiving data

int main()
{
    WSADATA wsaData;               // Structure used by WSAStartup function to init winsock on windows
    int server_fd, new_socket;     // File descriptors (Server Socket and Client Socket [new_socket])
    struct sockaddr_in address;    // Struct to hold server info (sin_family, sin_addr.s_addr, sin_port)
    int opt = 1;                   // int used to set options in the socket
    int addrlen = sizeof(address); // Length of address

    char recv_buffer[BUFFER_SIZE] = {0}; // Define buffer for receiving data

    const char *message = "Hello from PC!"; // Message to send to Pico

    // Initialize Winsock  MAKEWORD(2,2) specifies winsock version 2.2
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) // Check if winsoc struct is empty as well
    {
        printf("WSAStartup failed: %d\n", WSAGetLastError()); // Return error code if WSAStartup fails
        return 1;
    }

    // Create server socket, IPv4 (AF_INET), TCP (SOCK_STREAM)
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
    {
        printf("Socket failed: %d\n", WSAGetLastError()); // Get error code if socket creation fails
        WSACleanup();                                     // Clean up/free winsock resources
        return 1;
    }

    // Set socket options to allow reuse of addresses and ports
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) == SOCKET_ERROR)
    {
        printf("setsockopt failed: %d\n", WSAGetLastError()); // Get error code if setting socket operation fails
        closesocket(server_fd);                               // Close socket
        WSACleanup();                                         // Clean up/free winsock resources
        return 1;
    }

    // Init address struct for the server
    address.sin_family = AF_INET;         // IPv4
    address.sin_addr.s_addr = INADDR_ANY; // Accept conn on any network
    address.sin_port = htons(PORT);       // Convert port number to network byte order

    // Bind the socket to the address (IP + port)
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) == SOCKET_ERROR)
    {
        printf("Bind failed: %d\n", WSAGetLastError()); // Get error code if bind fails
        closesocket(server_fd);                         // Close socket
        WSACleanup();                                   // Clean up/free winsock resources
        return 1;
    }

    // Start listening for incoming connections
    if (listen(server_fd, 3) == SOCKET_ERROR)
    {
        printf("Listen failed: %d\n", WSAGetLastError()); // Get error code if listen fails
        closesocket(server_fd);                           // Close socket
        WSACleanup();                                     // Clean up/free winsock resources
        return 1;
    }

    printf("Waiting for a connection...\n");

    // Accept a connection
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, &addrlen)) == INVALID_SOCKET)
    {
        printf("Accept failed: %d\n", WSAGetLastError()); // Get error code if accept fails
        closesocket(server_fd);                           // Close coket
        WSACleanup();                                     // Clean up/free winsock resources
        return 1;
    }

    printf("Client connected. Waiting for message from client...\n");

    // Receive a message from the Pico W
    int bytes_received = recv(new_socket, recv_buffer, sizeof(recv_buffer) - 1, 0); // recv reads RAW data from a socket, last parameter is "flags" (types of msg transmission). Returns no. of bytes received if successful, else -1 and error no.

    if (bytes_received > 0)
    {
        // If message received, print
        recv_buffer[bytes_received] = '\0'; // Null-terminate the received data to print safely, because C strings are null-terminated
        printf("Message received from client: %s\n", recv_buffer);
    }

    else
    {
        // Print error if no data received/error occured
        printf("Failed to receive message from client\n");
    }

    // Send a message to the Pico W (client)
    if (send(new_socket, message, strlen(message), 0) < 0) // params: int socket, const void *buffer, size_t length, int flags. Returns no. of bytes sent if successful, else -1 and error no.
    {
        printf("Failed to send message to client\n");
    }

    else
    {
        printf("Message sent to client: %s\n", message);
    }

    // Sleep incase pico needs time to receive msg (can remove maybe)
    Sleep(2000);

    // Close the connection
    closesocket(new_socket);
    closesocket(server_fd);

    // Clean up/free winsock resources
    WSACleanup();

    return 0;
}
