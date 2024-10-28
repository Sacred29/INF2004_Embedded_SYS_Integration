#include "pico/stdlib.h"
#include <stdio.h>
#include "IR_DRIVER.h"

int main() {
               
    stdio_init_all(); 
    sleep_ms(10000);

    setup_ir(); 
    bool sampling_timer_function(__unused struct repeating_timer *t) ;
    setup_timer();
    

    while(1);
    return 0;
}