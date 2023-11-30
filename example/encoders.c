#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"

#include "encoders.h"

volatile uint32_t right_a_count = 0;
volatile uint32_t right_b_count = 0;
volatile uint32_t left_a_count = 0;
volatile uint32_t left_b_count = 0;

repeating_timer_t timer;

void encoder_callback(uint gpio, uint32_t events);
bool timer_callback(repeating_timer_t *rt);

void encoders_register_callbacks(void)
{
    printf("registering encoder callbacks\n");
    add_repeating_timer_us(-1 * MEASURE_PERIOD_SEC * SEC_TO_uS, timer_callback,NULL, &timer);

    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_A_PIN, GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_B_PIN, GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_A_PIN, GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_B_PIN, GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
}

void encoder_callback(uint gpio, uint32_t events)
{
  if (events & GPIO_IRQ_EDGE_FALL)
  {
    switch(gpio)
    {
        case RIGHT_ENCODER_A_PIN:
        right_a_count++;
        break;
        case RIGHT_ENCODER_B_PIN:
        right_b_count++;
        break;
        case LEFT_ENCODER_A_PIN:
        left_a_count++;
        break;
        case LEFT_ENCODER_B_PIN:
        left_b_count++;
        break;
    }
  }
}

void encoders_zero_distances(void)
{
    right_a_count=0;
    right_b_count=0;
    left_a_count=0;
    left_b_count=0;
}

inline float counts_to_mm(uint32_t count)
{
    return (float) count / COUNTS_PER_REV * WHEEL_CIRCUMFERENCE_MM * CALIB;
} 


float encoders_distance_traveled(void)
{
    float dist_ra, dist_rb, dist_la, dist_lb;
    dist_ra = counts_to_mm(right_a_count);
    dist_rb = counts_to_mm(right_b_count);
    dist_la = counts_to_mm(left_a_count);
    dist_lb = counts_to_mm(left_b_count);

    return (dist_ra + dist_rb + dist_la + dist_lb) / 4;
}

bool timer_callback(repeating_timer_t *rt)
{

  float dist_ra, dist_rb, dist_la, dist_lb;
  dist_ra = counts_to_mm(right_a_count);
  dist_rb = counts_to_mm(right_b_count);
  dist_la = counts_to_mm(left_a_count);
  dist_lb = counts_to_mm(left_b_count);

  int rpm_ra, rpm_rb, rpm_la, rpm_lb;
  rpm_ra = right_a_count / COUNTS_PER_REV * 60;
  rpm_rb = right_b_count / COUNTS_PER_REV * 60;
  rpm_la = left_a_count / COUNTS_PER_REV * 60;
  rpm_lb = left_b_count / COUNTS_PER_REV * 60;
//   uint32_t rpm = (count * (SECONDS_PER_MIN / MEASURE_PERIOD_SEC)) / COUNTS_PER_REV;

  printf("rpm: %d %d %d %d \n", rpm_ra, rpm_rb, rpm_la, rpm_lb);
  // printf("counts: %d %d %d %d \n", right_a_count, right_b_count, left_a_count, left_b_count);
  // printf("DISTS: %.1fmm %.1fmm %.1fmm %.1fmm \n", dist_ra, dist_rb, dist_la, dist_lb);

  encoders_zero_distances();
  return true; // keep repeating
}