#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "line_follower.h"

// Define GPIO pins Right Motor
#define RIGHT_PWM_PIN 2    // GP2 for PWM
#define RIGHT_DIR_PIN1 0   // GP0 for direction
#define RIGHT_DIR_PIN2 1   // GP1 for direction

// Define GPIO pins Left Motor
#define LEFT_PWM_PIN 10    // GP10 for PWM (updated from GP8)
#define LEFT_DIR_PIN1 6    // GP6 for direction
#define LEFT_DIR_PIN2 7    // GP7 for direction

#define ADC_SLEEP_MS 1

// Initialize button pin
const uint BTN_PIN = 21;
const uint BTN_PIN_INCREASE = 22;
const uint BTN_PIN_DECREASE = 20;

volatile bool move_signal = false;
volatile bool is_black = false;
volatile bool is_black_ready = false;


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

int main()
{
    stdio_init_all();

    
    sleep_ms(5000);
    printf("Hello Motor Control\n");
    printf("Press GP22 to start\n");

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

    // Initialize GPIO pins for Speed Control
    gpio_init(BTN_PIN_INCREASE);
    gpio_set_dir(BTN_PIN_INCREASE, GPIO_IN);
    gpio_pull_up(BTN_PIN_INCREASE);

    gpio_init(BTN_PIN_DECREASE);
    gpio_set_dir(BTN_PIN_DECREASE, GPIO_IN);
    gpio_pull_up(BTN_PIN_DECREASE);

    // Set up PWM on GPIO2 and GPIO10
    setup_pwm(RIGHT_PWM_PIN, 100.0f, 0.0f); // 100 Hz frequency, 0% duty cycle
    setup_pwm(LEFT_PWM_PIN, 100.0f, 0.0f);  // 100 Hz frequency, 0% duty cycle (updated to GPIO10)

    // Initialize button
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    // Baseline Duty Cycle
    float baseline_dc = 0.5;

    bool isMoving = false;
    setup_line_follower();
    struct repeating_timer sampling_timer;
    add_repeating_timer_ms(ADC_SLEEP_MS, line_follower_polling, NULL, &sampling_timer);
    

    // Set initial motor direction (forward)
    gpio_put(RIGHT_DIR_PIN1, 0);
    gpio_put(RIGHT_DIR_PIN2, 1);

    gpio_put(LEFT_DIR_PIN1, 0);
    gpio_put(LEFT_DIR_PIN2, 1);



    // Control motor direction and speed
    while (true)
    {
        if (is_black_ready) {
            is_black_ready = false;
            move_signal = is_black;
            printf("isBlack %d\n", is_black);
           
        }
        // Check if button is pressed to start movement
        if (!isMoving && !gpio_get(BTN_PIN_INCREASE))
        {
            
            isMoving = true;
            printf("GP22 pressed, starting movement\n");
            sleep_ms(550); // Debounce delay

            // Set initial duty cycle for both motors to start moving
            set_speed(0, RIGHT_PWM_PIN);
            set_speed(0, LEFT_PWM_PIN);
        }

        if (isMoving && is_black)
        {
            // Maintain duty cycle for both motors
            set_speed(baseline_dc, RIGHT_PWM_PIN);
            set_speed(baseline_dc, LEFT_PWM_PIN);
        }
        else if(isMoving && !is_black)
        {
            set_speed(0, RIGHT_PWM_PIN);
            set_speed(0.25, LEFT_PWM_PIN);
        }

        // sleep_ms(500); // Adjust polling rate as needed
    }

    return 0;
}
