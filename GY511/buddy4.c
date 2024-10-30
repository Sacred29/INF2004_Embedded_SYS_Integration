#include <stdio.h>
#include "pico/stdlib.h"
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
#define MAX_TILT 16000      // Maximum tilt value for full speed
#define MIN_TILT 2000       // Minimum tilt value to start moving (any tilt below this is ignored)
#define SMA_WINDOW_SIZE 5   // Size of the moving average window  



// Moving average buffers
int16_t x_buffer[SMA_WINDOW_SIZE] = {0};
int16_t y_buffer[SMA_WINDOW_SIZE] = {0};
int index = 0;




void send_message_to_pico(const char *message) {
    printf("%s\n", message); 
    // logic for sending the message to the other Pico ,add below
}

// Initialize I2C and accelerometer pins
void i2c_init_pins() {
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
}

// Write to register
void write_register(uint8_t addr, uint8_t reg, uint8_t data) {
    uint8_t buf[] = {reg, data};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}

// Read from register
uint8_t read_register(uint8_t addr, uint8_t reg) {
    uint8_t data;
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, &data, 1, false);
    return data;
}

// Read 16-bit axis data
int16_t read_axis(uint8_t reg_l, uint8_t reg_h) {
    uint8_t lsb = read_register(ACCEL_ADDR, reg_l);
    uint8_t msb = read_register(ACCEL_ADDR, reg_h);
    return (int16_t)((msb << 8) | lsb);
}

// Simple Moving Average (SMA) filter
int16_t apply_sma_filter(int16_t *buffer, int16_t new_value) {
    buffer[index] = new_value;
    int32_t sum = 0;
    for (int i = 0; i < SMA_WINDOW_SIZE; i++) {
        sum += buffer[i];
    }
    return (int16_t)(sum / SMA_WINDOW_SIZE);
}

//map the tilt level as speed
int map_tilt_to_speed(int16_t tilt_value) {
    int tilt_magnitude = abs(tilt_value);

    if (tilt_magnitude < MIN_TILT) {
        return 0;  // Below the threshold, no movement
    }

    
    int speed_percent = (tilt_magnitude - MIN_TILT) * 100 / (MAX_TILT - MIN_TILT);

    // Clip to 100% maximum if value is very close
    if (speed_percent > 100) {
        speed_percent = 100;
    }
    return speed_percent;
}

// Determine and print tilt-based speed percentage
void send_tilt_speed(int16_t filtered_x, int16_t filtered_y) {
    int forward_speed = map_tilt_to_speed(filtered_y);
    int sideways_speed = map_tilt_to_speed(filtered_x);

    if (filtered_y < -MIN_TILT) {
        char message[50];
        snprintf(message, sizeof(message), "Forward at %d%% Speed", forward_speed);
        send_message_to_pico(message);
    } else if (filtered_y > MIN_TILT) {
        char message[50];
        snprintf(message, sizeof(message), "Backward at %d%% Speed", forward_speed);
        send_message_to_pico(message);
    }

    if (filtered_x < -MIN_TILT) {
        char message[50];
        snprintf(message, sizeof(message), "Right at %d%% Speed", sideways_speed);
        send_message_to_pico(message);
    } else if (filtered_x > MIN_TILT) {
        char message[50];
        snprintf(message, sizeof(message), "Left at %d%% Speed", sideways_speed);
        send_message_to_pico(message);
    }

    if (forward_speed == 0 && sideways_speed == 0) {
        send_message_to_pico("Stop");
    }
}

int main() {
    stdio_init_all();
    i2c_init_pins();

    // Initialize accelerometer with 100Hz data rate, enable X, Y, Z axes (refer to docs 0x57)
    write_register(ACCEL_ADDR, CTRL_REG1_A, 0x57);

    while (true) {
        // Read raw accelerometer data
        int16_t accel_x = read_axis(OUT_X_L_A, OUT_X_H_A);
        int16_t accel_y = read_axis(OUT_Y_L_A, OUT_Y_H_A);

        // Apply SMA filtering
        int16_t filtered_x = apply_sma_filter(x_buffer, accel_x);
        int16_t filtered_y = apply_sma_filter(y_buffer, accel_y);

        // Move index for circular buffer
        index = (index + 1) % SMA_WINDOW_SIZE;

        // Determine tilt direction 
        send_tilt_speed(filtered_x, filtered_y);

        sleep_ms(100);  // Small delay 
    }

    return 0;
}