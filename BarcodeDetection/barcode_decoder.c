// barcode_decoder
#include "barcode_decoder.h"     
#include <stdio.h>
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <math.h>

void setup_adc();
bool sampling_timer_function(__unused struct repeating_timer *t);
void classify_bar(uint32_t duration_ms, bool is_black, uint32_t adc_reading, uint32_t contrast_threshold);
void barcode_check(uint32_t index, bool* isBlack, bool* isWide);
bool check_checksum(uint32_t start_index, bool* isWide, const ChecksumPattern* checksum_pattern, bool reverse);

void setup_adc()
{
    adc_init();                              // ADC library
    gpio_disable_pulls(ADC_GPIO);            // Disable Button thing for GPIO PIN used for ADC [In the button, it auto sets voltage to high/low]
                                             // It apparently resistors can make voltage increase as well.
    gpio_set_input_enabled(ADC_GPIO, false); // Disable Digital input from IR sensor, continous voltage readings
    adc_select_input(ADC_Input);
}

bool barcode_polling_function(__unused struct repeating_timer *t)
{
    uint32_t reading = adc_read();

    static uint32_t data[NUM_SLOT] = {0};
    static uint32_t index = 0;
    static uint64_t sum = 0;
    static uint32_t count = 0;

    static uint32_t adc_reading;

    if (count < NUM_SLOT)
        count++; // Increment count till it reaches NUM_SLOT

    // Moving Average Calculation
    sum -= data[index];             // Subtract the oldest element from sum
    data[index] = reading;          // Add new element to the data array
    sum += data[index];             // Add the new element to sum
    index = (index + 1) % NUM_SLOT; // Update index for circular buffer

    // Calculate average contrast
    uint32_t average_contrast = (uint32_t)ceil((double)sum / count);

    adc_reading = (uint32_t)ceil((double)sum / count);

    static uint32_t min_threshold = 4095;
    static uint32_t max_threshold = 0;
    static uint32_t contrast_threshold = 4095;

    if (adc_reading < min_threshold)
    {
        min_threshold = adc_reading;
        contrast_threshold = (uint32_t)ceil((double)(min_threshold + max_threshold) / 2);
    }
    if (adc_reading > max_threshold)
    {
        max_threshold = adc_reading;
        contrast_threshold = (uint32_t)ceil((double)(min_threshold + max_threshold) / 2);
    }

    static bool last_color_black = false;
    static uint32_t count_time = 0;
    bool current_color_black = (adc_reading >= contrast_threshold);

    if (current_color_black != last_color_black)
    {

        classify_bar(count_time, last_color_black, adc_reading, contrast_threshold);
        count_time = 0;
        last_color_black = current_color_black;
    }
    else
    {
        count_time++;
    }

    return true;
}

void classify_bar(uint32_t duration_ms, bool is_black, uint32_t adc_reading, uint32_t contrast_threshold)
{
    // printf("Black: %d, Duration %u, reading: %u, threshold: %u\n", is_black, duration_ms, adc_reading, contrast_threshold);

    static uint32_t durations[BARCODE_BUFFER] = {0};
    static bool isBlack[BARCODE_BUFFER] = {0};
    static bool isWide[BARCODE_BUFFER] = {0};
    static uint32_t index = 0;
    static uint32_t count = 0;
    static uint32_t total_duration = 0; // Running total for moving average

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
        uint32_t width_threshold = (uint32_t)ceil((double)(total_duration / BARCODE_BUFFER) * 1.33);

        uint32_t i = index;

        do
        {
            isWide[i] = (durations[i] > width_threshold);
            i = (i + 1) % BARCODE_BUFFER; // Move to the next element in a circular manner
        } while (i != index); // Loop until we've completed one full pass

        
        // barcode check
        barcode_check( index, isBlack, isWide);

    }

    index = (index + 1) % BARCODE_BUFFER;
}

// Define a function to check if a given sequence matches a specified checksum pattern in
// either normal or reverse order.
bool check_checksum(uint32_t start_index, bool* isWide, const ChecksumPattern* checksum_pattern, bool reverse) {
    for (uint32_t count = 0; count < BARCODE_BUFFER - 1; ++count) {
        uint32_t checksum_index = reverse ? (BARCODE_BUFFER - 2 - count) : count;
        if (isWide[(start_index + count) % BARCODE_BUFFER] != checksum_pattern->pattern[checksum_index]) {
            return false;
        }
    }
    return true;
}

// Main function to check for a matching checksum pattern and print the corresponding character.
void barcode_check(uint32_t index, bool* isBlack, bool* isWide) {
    uint32_t read_index = (index + 1) % BARCODE_BUFFER;

    // Define checksum patterns and associated characters.
    ChecksumPattern patterns[] = {
        {{false, false, true, false, true, false, false, true, false}, '*'},
        {{true, false, false, true, false, false, false, false, true }, 'A'},
        {{false, false, false, true, true, false, true, false, false}, 'F'},
        {{false, false, false, false, true, false, true, true, false}, 'Z'},
        // Add more patterns here as needed
    };

    // Number of patterns defined in the array.
    size_t num_patterns = sizeof(patterns) / sizeof(patterns[0]);

    if (isBlack[read_index]) {
        for (size_t i = 0; i < num_patterns; ++i) {
            if (check_checksum(read_index, isWide, &patterns[i], false)) {
                // Send wifi message
                return;
            } else if (check_checksum(read_index, isWide, &patterns[i], true)) {
                // Send Wifi Message
                return;
            }
        }
        
    }
}