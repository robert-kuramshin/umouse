
#include <stdio.h>
#include <stdlib.h>

#include "pico/bootrom.h"
#include "logflash.h"
#include "pico/cyw43_arch.h"

#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    // if (cyw43_arch_init()) {
    //     printf("Wi-Fi init failed");
    //     return -1;
    // }

    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(250);
    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(250);

    sleep_ms(5000);
    int res = init_log_flash();
    if (res != 0)
    {
        printf("could not init logflash with %d\n",res);
        return 1;
    }

    printf("reading all: \n\n");

    print_all();

    printf("\ndone reading all\n");

    reset_usb_boot(0, 0);
}