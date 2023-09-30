/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#define MOTOR_A_PIN (16)
#define MOTOR_B_PIN (17)
#define ENCODER_A_PIN (18)
#define ENCODER_B_PIN (19)

#define WRAP (50000)

#define COUNTS_PER_REV (3)
#define SECONDS_PER_MIN (60)
#define MEASURE_PERIOD_SEC (3)
#define SEC_TO_uS (1000000)


volatile uint32_t counter = 0;


// void gpio_callback_a(uint gpio, uint32_t events) {
//     printf("event A\n");
// }

void gpio_callback_b(uint gpio, uint32_t events) {
    // printf("event B\n");
    if (events & GPIO_IRQ_EDGE_FALL) {
        counter += 1;
    }
}

bool timer_callback(repeating_timer_t *rt) {
    uint32_t count = counter;
    counter = 0;

    uint32_t rpm = (count * (SECONDS_PER_MIN/MEASURE_PERIOD_SEC)) / COUNTS_PER_REV;

    printf("RPM: %d COUNT: %d \n", rpm, count);
    return true; // keep repeating
}


int main() {
    stdio_init_all();

    // Init motor output
    gpio_set_function(MOTOR_A_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(MOTOR_A_PIN);
    uint chan = pwm_gpio_to_channel (MOTOR_A_PIN);
    pwm_set_enabled (slice, true);
    pwm_set_wrap(slice, WRAP);
    printf("Freq %d\n", 125000000/WRAP);

    //Init interrupts
    // gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_LEVEL_HIGH, true, &gpio_callback_a);
    gpio_set_irq_enabled_with_callback(ENCODER_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_b);

    //Repeating timer
    repeating_timer_t timer;
    add_repeating_timer_us(-1 * MEASURE_PERIOD_SEC * SEC_TO_uS ,timer_callback,NULL, &timer);

    // while (true) {
    //     char c = getchar_timeout_us(100);
    //     if (c < 0 || c>=255) {
    //         continue;
    //     }
    //     int digit = c - 48;
    //     if (digit < 0 || digit > 10) {
    //         continue;
    //     }
    //     uint level =  digit * (WRAP/10);
    //     printf("Setting duty cycle to %d/%d\n", level, WRAP);
    //     pwm_set_chan_level(slice, chan, level);
    // }

    sleep_us(10 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 1 * (WRAP/10));
    sleep_us(3 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 2 * (WRAP/10));
    sleep_us(3 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 3 * (WRAP/10));
    sleep_us(3 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 4 * (WRAP/10));
    sleep_us(3 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 5 * (WRAP/10));
    sleep_us(3 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 6 * (WRAP/10));
    sleep_us(3 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 7 * (WRAP/10));
    sleep_us(3 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 8 * (WRAP/10));
    sleep_us(3 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 9 * (WRAP/10));
    sleep_us(3 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 10 * (WRAP/10));
    sleep_us(3 *1000 * 1000);
    pwm_set_chan_level(slice, chan, 0);
}
