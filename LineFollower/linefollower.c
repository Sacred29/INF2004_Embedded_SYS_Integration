#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"
#include <math.h>

#define ADC_GPIO 26
#define ADC_Input 0
#define ADC_SLEEP_MS 25
#define CONTRAST_THRESHOLD 4050 // White is lower value, so brighter need to reduce threshold.
#define mbaTASK_MESSAGE_BUFFER_SIZE (60)
#define NUM_SLOT 3

volatile bool last_black = true;
volatile int time;

static MessageBufferHandle_t xSampledDataBuffer;


void vLaunch(void);
void sample_task(__unused void *params);
void moving_task(__unused void *params);

bool sampling_timer_function(__unused struct repeating_timer *t);



int main()
{
    stdio_init_all();                        // PrintF Library
    adc_init();                              // ADC library
    gpio_disable_pulls(ADC_GPIO);            // Disable Button thing for GPIO PIN used for ADC [In the button, it auto sets voltage to high/low]
                                             // It apparently resistors can make voltage increase as well.
    gpio_set_input_enabled(ADC_GPIO, false); // Disable Digital input from IR sensor, continous voltage readings
    adc_select_input(ADC_Input);             // depends on the GPIO PIN 26-28 or temperature [0-3]
    

    struct repeating_timer sampling_timer;
    add_repeating_timer_ms(ADC_SLEEP_MS, sampling_timer_function, NULL, &sampling_timer);

    vLaunch();
    return 0;
}



void vLaunch(void)
{

    TaskHandle_t movingTask;
    xTaskCreate(moving_task, "MovingAverageTask", configMINIMAL_STACK_SIZE, NULL, 2, &movingTask);

    xSampledDataBuffer = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}   

void moving_task(__unused void *params)
/*Calculates moving average of sampled results*/
{
    uint32_t fReceivedData;
    size_t xReceivedBytes;
    uint32_t data[NUM_SLOT] = {0};
    int index = 0;
    int count = 0;
    int sum = 0;

    while (true)
    {
        xReceivedBytes = xMessageBufferReceive(
            xSampledDataBuffer,   /* The message buffer to receive from. */
            (void *)&fReceivedData, /* Location to store received data. */
            sizeof(fReceivedData),  /* Maximum number of bytes to receive. */
            portMAX_DELAY);

        sum -= data[index];          // Subtract the oldest element from sum
        data[index] = fReceivedData; // Assign the new element to the data
        sum += data[index];          // Add the new element to sum
        index = (index + 1) % NUM_SLOT;    // Update the index- make it circular

        if (count < NUM_SLOT)
            count++; // Increment count till it reaches Num Slots

        uint32_t average_contrast = ceil(sum / count);
        
        if (last_black == true && average_contrast <= CONTRAST_THRESHOLD) {
            // Change to white
            printf("Time on Black before changing to white: %i, %u\n", time, average_contrast);

            time = 0;
            last_black = false;
        }
        else if (last_black == false && average_contrast > CONTRAST_THRESHOLD) {
            // Change to black
            printf("Time on White before changing to Black: %i, %u\n", time, average_contrast);
            time = 0;
            last_black = true;
        }
        else {
            time += ADC_SLEEP_MS; // sampling time.
        }

    }
}

bool sampling_timer_function(__unused struct repeating_timer *t) {
    uint32_t reading = adc_read();
    xMessageBufferSend(/* The message buffer to write to. */
                           xSampledDataBuffer,
                           /* The source of the data to send. */
                           (void *)&reading,
                           /* The length of the data to send. */
                           sizeof(reading),
                           /* The block time; 0 = no block */
                           0);
    return true;
}