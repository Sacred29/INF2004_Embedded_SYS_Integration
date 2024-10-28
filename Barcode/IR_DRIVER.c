#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/adc.h"
#include <math.h>

#define GPIO_PIN 26
#define ADC_NUM 0
#define ADC_SLEEP_MS 25
#define WINDOW_SIZE 3

void setup_ir();
void setup_timer();
bool sampling_timer_function(__unused struct repeating_timer *t) ;

void setup_ir()
{
    adc_init();                              // ADC library
    gpio_disable_pulls(GPIO_PIN);            // Disable Button thing for GPIO PIN used for ADC [In the button, it auto sets voltage to high/low]
                                      // It apparently resistors can make voltage increase as well.
    gpio_set_input_enabled(GPIO_PIN, false); // Disable Digital input from IR sensor, continous voltage readings
    adc_select_input(ADC_NUM);               // depends on the GPIO PIN 26-28 or temperature [0-3] 
}


bool sampling_timer_function(__unused struct repeating_timer *t) {
    printf("WHAT\n");
    return true;
    // static uint32_t adc_buffer[WINDOW_SIZE] = {0}; // Buffer to store the last 3 ADC readings
    // static int index = 0;                // Current index for buffer replacement
    // static int sum = 0;                  // Sum of values in the buffer
    // static uint8_t count = 0;

    // // Remove the oldest reading from the sum, add the new reading
    // sum -= adc_buffer[index];
    // adc_buffer[index] = adc_read();
    // sum += adc_buffer[index];
    // if (count < WINDOW_SIZE) count++;

    // // Update the moving average
    // // printf("Average: %i\n", ceil(sum / count));
    // printf("hi");

    // // Update the index to point to the next position
    // index = (index + 1) % WINDOW_SIZE;
}
void setup_timer() 
{
    printf("hello\n");
    struct repeating_timer sampling_timer;
    bool timer_initialized = add_repeating_timer_ms(100, sampling_timer_function, NULL, &sampling_timer);
    printf("WALAO\n");
    if(!timer_initialized) {printf("KNN\n");}
    else{printf( "???\n" );}

}
