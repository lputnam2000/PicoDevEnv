#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <string.h>
#include <lwip/sockets.h>

#define WIFI_SSID "your_SSID"
#define WIFI_PASSWORD "your_password"
#define SERVER_IP "192.168.1.17"
#define SERVER_PORT 12345

int main() {
    stdio_init_all();

    // Initialize the Cyw43 Wi-Fi driver
    if (cyw43_arch_init()) {
        printf("Wi-Fi initialization failed!\n");
        return -1;
    }

    // Connect to Wi-Fi
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_blocking(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK)) {
        printf("Wi-Fi connection failed!\n");
        return -1;
    }
    printf("Connected to Wi-Fi!\n");

    // Set up socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        printf("Failed to create socket\n");
        return -1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("Failed to connect to server\n");
        close(sock);
        return -1;
    }
    printf("Connected to server!\n");

    // Send and receive data
    char message[] = "Hello, server!";
    send(sock, message, strlen(message), 0);

    char buffer[1024] = {0};
    int len = recv(sock, buffer, sizeof(buffer), 0);
    printf("Received: %s\n", buffer);

    // Close the socket
    close(sock);

    // Deinitialize Wi-Fi
    cyw43_arch_deinit();

    return 0;
}
