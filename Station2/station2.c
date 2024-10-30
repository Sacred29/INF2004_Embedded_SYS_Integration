#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"
#include <math.h>
#include <stdint.h>
#include "line_follower.h" 
#include "motor_control.h"
#include "config.h"

// Initialize button pin
const uint BTN_PIN = 21;
const uint BTN_PIN_INCREASE = 22;
const uint BTN_PIN_DECREASE = 20;

#define ADC_SLEEP_MS 10

volatile bool is_black_ready = false;  
volatile bool is_black = false;  



int main() {
    stdio_init_all(); 
    sleep_ms(10000);
    printf("Start\n");
    setup_line_follower();
    
    // Set up PWM on GPIO2 and GPIO8
    setup_pwm(RIGHT_PWM_PIN, 100.0f, 0.5f); // 100 Hz frequency, 50% duty cycle
    setup_pwm(LEFT_PWM_PIN, 100.0f, 0.5f); 
    
    setup_motor_direction(RIGHT_DIR_PIN1, RIGHT_DIR_PIN2);
    setup_motor_direction(LEFT_DIR_PIN1, LEFT_DIR_PIN2);
    initialize_speed_change_button(BTN_PIN_INCREASE);
    initialize_speed_change_button(BTN_PIN_DECREASE);

    struct repeating_timer sampling_timer;
    bool success = add_repeating_timer_ms(ADC_SLEEP_MS, line_follower_polling, NULL, &sampling_timer);
    
    while(1) {
        if (is_black_ready) {              // Check if new ADC data is ready
            is_black_ready = false;        // Reset the flag
            printf("isBlack: %u\n", is_black);  // Print the shared ADC reading
            
            //move forward
            gpio_put(RIGHT_DIR_PIN1, 0);
            gpio_put(RIGHT_DIR_PIN2, 1);

            gpio_put(LEFT_DIR_PIN1, 1);
            gpio_put(LEFT_DIR_PIN2, 0);
        }
    }
    printf("END\n");
    return 0;
}


