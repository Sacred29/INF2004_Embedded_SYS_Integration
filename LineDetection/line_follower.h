// line_follower.h
#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "pico/stdlib.h"
#include <stdint.h>
#include <stdbool.h>

// ADC and GPIO configuration
#define ADC_GPIO 28
#define ADC_Input 2
#define NUM_SLOT 3


// Function declarations
void setup_line_follower(void);
bool line_follower_polling(__unused struct repeating_timer *t);

#endif // LINE_FOLLOWER_H
