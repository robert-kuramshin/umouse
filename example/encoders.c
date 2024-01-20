#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"

#include "encoders.h"

volatile uint32_t right_count = 0;
volatile uint32_t left_count = 0;

repeating_timer_t timer;

void encoder_callback(uint gpio, uint32_t events);
bool timer_callback(repeating_timer_t *rt);

void encoders_register_callbacks(void)
{
    printf("registering encoder callbacks\n");
    add_repeating_timer_us(-1 * MEASURE_PERIOD_SEC * SEC_TO_uS, timer_callback,NULL, &timer);

    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_A_PIN, GPIO_IRQ_MASK, true, &encoder_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_B_PIN, GPIO_IRQ_MASK, true, &encoder_callback);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_A_PIN, GPIO_IRQ_MASK, true, &encoder_callback);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_B_PIN, GPIO_IRQ_MASK, true, &encoder_callback);
}

void encoder_callback(uint gpio, uint32_t events)
{
  if (events & GPIO_IRQ_MASK)
  {
    switch(gpio)
    {
        case RIGHT_ENCODER_A_PIN:
        right_count++;
        break;
        case RIGHT_ENCODER_B_PIN:
        right_count++;
        break;
        case LEFT_ENCODER_A_PIN:
        left_count++;
        break;
        case LEFT_ENCODER_B_PIN:
        left_count++;
        break;
    }
  }
}

void encoders_zero_distances(void)
{
    right_count=0;
    left_count=0;
}

inline float counts_to_mm(uint32_t count)
{
    return (float) count / COUNTS_PER_REV * WHEEL_CIRCUMFERENCE_MM * CALIB;
} 


float encoders_distance_traveled(void)
{
    float dist_r, dist_l;
    dist_r = counts_to_mm(right_count);
    dist_l = counts_to_mm(left_count);

    return (dist_r + dist_l) / 2;
}

bool timer_callback(repeating_timer_t *rt)
{

  float dist_r, dist_l;
  dist_r = counts_to_mm(right_count);
  dist_l = counts_to_mm(left_count);

  int rpm_r, rpm_l;
  rpm_r = right_count / COUNTS_PER_REV * (SECONDS_PER_MIN/MEASURE_PERIOD_SEC);
  rpm_l = left_count / COUNTS_PER_REV * (SECONDS_PER_MIN/MEASURE_PERIOD_SEC);

  printf("rpm: %d %d \n", rpm_r, rpm_l);
  printf("counts: %ld %ld \n", right_count, left_count);
  printf("DISTS: %.1fmm %.1fmm \n", dist_r, dist_l);

  // encoders_zero_distances();
  return true; // keep repeating
}