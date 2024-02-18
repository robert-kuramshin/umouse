
#include <stdio.h>
#include <stdlib.h>

#include "hardware/rtc.h"
#include "logflash.h"
#include "pico/util/datetime.h"
#include "pico/cyw43_arch.h"
#include "pico/bootrom.h"

#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(250);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(250);

    sleep_ms(5000);

    char datetime_buf[256];
    char *datetime_str = &datetime_buf[0];
    datetime_t t = {
            .year  = 2000,
            .month = 1,
            .day   = 1,
            .dotw  = 0, // 0 is Sunday, so 5 is Friday
            .hour  = 0,
            .min   = 0,
            .sec   = 00
    };

    // Start the RTC
    rtc_init();
    rtc_set_datetime(&t);

    rtc_get_datetime(&t);
    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
    printf("Inited date time to: %s\n", datetime_str);

    int res = init_log_flash();
    if (res != 0)
    {
        printf("could not init logflash with %d\n",res);
        return 1;
    }

    printf("sizeof(page_t)=%d\n",sizeof(page_t));
    printf("num_pages=%d\n",NUM_PAGES);
    printf("header_offset=%x\n",FLASH_LOG_HEADER_OFFSET);
    printf("page_offset=%x\n",FLASH_LOG_START_OFFSET);

    printf("printing all:\n");

    print_all();

    printf("\n\ndone\n");

    printf("erasing\n");
    erase_all();
    printf("done");

    rtc_get_datetime(&t);
    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
    printf("%s\n",datetime_str);
    datetime_t start = t;

    int num_lines=500;
    char *data = malloc(82);
    sprintf(data,"ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc");
    int min_size = 1;
    int max_size = 80;
    int cur_size = min_size;
    for (int i = 0; i < num_lines; i++)
    {
        // printf("writing line %d\n",i);
        data[cur_size]='\0';
        lfprintf("%04d: %s\n",i,data);
        data[cur_size]='c';
        if (cur_size < max_size)
        {
            cur_size += 5;
        } else {
            cur_size = min_size;
        }
    }
    rtc_get_datetime(&t);
    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
    printf("%s\n",datetime_str);

    int seconds = (t.min - start.min)*60 + (t.sec - start.sec);
    printf("Took %d seconds\n", seconds);
    printf("Per line: %dms\n", (seconds * 1000) / num_lines);
    reset_usb_boot(0, 0);
}