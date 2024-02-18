#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"

#include "encoders.h"

int main() {
    stdio_init_all();
    sleep_ms(5000);
    encoders_register_callbacks();
    encoders_register_timer_callback(2);
    while(true)
    {
        sleep_ms(100);
    }
}