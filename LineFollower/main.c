// main.c
#include "pico/stdlib.h"
#include <stdio.h>
#include "IR_DRIVER.h"

#define GPIO_PIN 26
#define ADC_NUM 0

uint32_t adc_value = 0;

int main() {
                          // PrintF Library
    stdio_init_all();  
    setup_ir(GPIO_PIN, ADC_NUM);
    
    
    while (1) {
        printf("oii: %u\n", adc_value);
        sleep_ms(500);
    };
    return 0;
}
