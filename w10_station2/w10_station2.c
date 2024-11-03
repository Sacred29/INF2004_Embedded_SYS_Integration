#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

// Wifi
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h" // Maybe need (need to test)
#include "lwip/sockets.h"  // lwIP socket API for networking
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif
#define configMINIMAL_STACK_SIZE (2048)

#define configCHECK_FOR_STACK_OVERFLOW 2 // Enable stack overflow checking

// #define SERVER_IP "172.20.10.5" // PC/Server IP address
#define SERVER_IP "192.168.1.81"
#define SERVER_PORT 65431 // Port no. the server is listening on
#define WIFI_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define WIFI_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 2048)
#define SENSOR_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define SENSOR_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 2048)

// Message buffer
MessageBufferHandle_t xControlWifiMessageBuffer;
#define WIFI_MESSAGE_BUFFER_SIZE (60)

#define BARCODE_BUFFER 10
#define BTN_PIN 21
#define BTN_PIN_INCREASE 22
// Define GPIO pins Right Motor
#define RIGHT_PWM_PIN 2  // GP2 for PWM
#define RIGHT_DIR_PIN1 0 // GP0 for direction
#define RIGHT_DIR_PIN2 1 // GP1 for direction

// Define GPIO pins Left Motor
#define LEFT_PWM_PIN 10 // GP10 for PWM (updated from GP8)
#define LEFT_DIR_PIN1 6 // GP6 for direction
#define LEFT_DIR_PIN2 7 // GP7 for direction

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
    {'~', {false, false, false, false, false, false, false, false, false}},
    {'~', {false, false, false, false, false, false, false, false, false}},

    {'~', {false, false, false, false, false, false, false, false, false}},
    // Add more characters and their corresponding patterns
};

// IR Prototypes
void line_follower(uint16_t adc_value, bool isMoving, float baseline_dc);
void barcode_detector(uint16_t adc_value);
void classify_bar(uint16_t duration_ms, bool is_black, uint16_t adc_reading);
void barcode_start_check(uint8_t index, bool *isBlack, bool *isWide, uint8_t *start_index, uint8_t *bar_count, bool *started_reading, bool *is_forward, char *message);
void barcode_read_char(bool *isWide, uint8_t *start_index, uint8_t *bar_count, bool *started_reading, bool *is_forward, char *message);
bool adc_timer_callback(repeating_timer_t *rt);
void init_adc_timer();
void process_adc_data(bool isMoving, float baseline_dc);

// Motor Prototypes
void setup_motor();
void setup_buttons();
void setup_pwm(uint gpio, float freq, float duty_cycle);
void set_speed(float duty_cycle, uint gpio_pin);

void add_char_to_string(char *message, char new_char);

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

void sensor_task(__unused void *params)
{

    while (true)
    {
        // handle adc data
        if (data_ready)
        {
            process_adc_data(isMoving, baseline_dc);
            data_ready = false; // Reset the flag after processing
        }

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

int main()
{
    stdio_init_all();
    setup_motor();
    setup_buttons();
    sleep_ms(5000);
    printf("Hello Week 10 Station2 Control\n");
    printf("Press GP22 to start\n");

    // start adc repeating timers
    init_adc_timer();

    // Motor variables
    // Baseline Duty Cycle
    float baseline_dc = 0.77;
    bool isMoving = false;

    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;
    rtos_name = "FreeRTOS";
    printf("Starting %s on core 0:\n", rtos_name);

    while (true)
    {
        // Check if button is pressed to start movement
        if (!isMoving && !gpio_get(BTN_PIN_INCREASE))
        {
            isMoving = true;
            printf("GP22 pressed, starting movement\n");
            sleep_ms(20); // Debounce delay

            // Set initial duty cycle for both motors to start moving
            set_speed(0, RIGHT_PWM_PIN);
            set_speed(0, LEFT_PWM_PIN);
        }
        vLaunch(); // Launch main task
        // handle motor logic
    }
    return 0;
}

void line_follower(uint16_t adc_value, bool isMoving, float baseline_dc)
{
    if (isMoving)
    {
        // black detected
        if (adc_value >= contrast_thresh)
        {

            // Move Forward
            // Maintain duty cycle for both motors
            set_speed(baseline_dc, RIGHT_PWM_PIN);
            set_speed(baseline_dc, LEFT_PWM_PIN);
        }
        // white detected
        else
        {
            // Correct motor
            // printf("MOVE CORRECTION\n");
            set_speed(0, RIGHT_PWM_PIN);
            set_speed(0.4, LEFT_PWM_PIN);
        }
    }
}
// Timer callback to sample ADCs
bool adc_timer_callback(repeating_timer_t *rt)
{
    // Read ADC0 and ADC1
    adc_select_input(1);
    adc0_data = adc_read();

    adc_select_input(2);
    adc1_data = adc_read();

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
    // printf("Black: %d, Duration %u, reading: %u, threshold: %u\n", is_black, duration_ms, adc_reading, contrast_threshold);

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
                    // Ensure it stops reading.
                    // Sometimes the start reading function does not catch
                    if (barcode_characters[char_idx].character == '*')
                    {
                        *started_reading = false;
                        message[strlen(message) - 1] = '\0';
                        printf("SEND TO WIFI\n");
                        // send message here
                        xMessageBufferSend(xControlWifiMessageBuffer, (void *)message, strlen(message), 0);
                        printf("Message: %s\n", message);
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
void process_adc_data(bool isMoving, float baseline_dc)
{
    // adc data values are global volatile
    line_follower(adc0_data, isMoving, baseline_dc);

    barcode_detector(adc1_data);
}

// Motor Functions

void setup_motor()
{
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

    // Set up PWM on GPIO2 and GPIO10
    setup_pwm(RIGHT_PWM_PIN, 100.0f, 0.0f); // 100 Hz frequency, 0% duty cycle
    setup_pwm(LEFT_PWM_PIN, 100.0f, 0.0f);  // 100 Hz frequency, 0% duty cycle (updated to GPIO10)

    // Set initial motor direction (forward)
    gpio_put(RIGHT_DIR_PIN1, 0);
    gpio_put(RIGHT_DIR_PIN2, 1);

    gpio_put(LEFT_DIR_PIN1, 0);
    gpio_put(LEFT_DIR_PIN2, 1);
}

void setup_buttons()
{
    // Initialize button
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    // Initialize GPIO pins for Speed Control
    gpio_init(BTN_PIN_INCREASE);
    gpio_set_dir(BTN_PIN_INCREASE, GPIO_IN);
    gpio_pull_up(BTN_PIN_INCREASE);
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