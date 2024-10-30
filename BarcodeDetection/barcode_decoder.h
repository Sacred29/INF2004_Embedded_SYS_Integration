// barcode_decoder.h
#ifndef BARCODE_DECODER_H
#define BARCODE_DECODER_H

#include "pico/stdlib.h"
#include <stdint.h>
#include <stdbool.h>

// ADC and GPIO configuration
#define ADC_GPIO 27
#define ADC_Input 1
#define NUM_SLOT 3
#define BARCODE_BUFFER 10

typedef struct {
    bool pattern[BARCODE_BUFFER - 1];
    char character;
} ChecksumPattern;

// Function declarations specific to barcode decoding
void setup_barcode_decoder(void);
bool barcode_polling_function(__unused struct repeating_timer *t);

#endif // BARCODE_DECODER_H
