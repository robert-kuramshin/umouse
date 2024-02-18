#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "encoders.h"

repeating_timer_t timer;

volatile uint32_t right_count = 0;
volatile uint32_t left_count = 0;

int measure_period = MEASURE_PERIOD_SEC;

void encoder_callback(uint gpio, uint32_t events);
bool timer_callback(repeating_timer_t *rt);

void encoders_register_callbacks(void)
{
    gpio_init(RIGHT_ENCODER_A_PIN);
    gpio_init(RIGHT_ENCODER_B_PIN);
    gpio_init(LEFT_ENCODER_A_PIN);
    gpio_init(LEFT_ENCODER_B_PIN);
    gpio_set_dir(RIGHT_ENCODER_A_PIN, GPIO_IN);   
    gpio_set_dir(RIGHT_ENCODER_B_PIN, GPIO_IN);   
    gpio_set_dir(LEFT_ENCODER_A_PIN, GPIO_IN);   
    gpio_set_dir(LEFT_ENCODER_B_PIN, GPIO_IN);   
    gpio_pull_up(RIGHT_ENCODER_A_PIN);
    gpio_pull_up(RIGHT_ENCODER_B_PIN);
    gpio_pull_up(LEFT_ENCODER_A_PIN);
    gpio_pull_up(LEFT_ENCODER_B_PIN);
    printf("registering encoder callbacks\n");

    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_A_PIN, GPIO_IRQ_MASK, true, &encoder_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_B_PIN, GPIO_IRQ_MASK, true, &encoder_callback);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_A_PIN, GPIO_IRQ_MASK, true, &encoder_callback);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_B_PIN, GPIO_IRQ_MASK, true, &encoder_callback);
}

void encoders_register_timer_callback(int period_sec)
{
    measure_period = period_sec;
    add_repeating_timer_us(-1 * measure_period * SEC_TO_uS, timer_callback,NULL, &timer);
}

enum encoder_sig {
  ARISE = 1,
  BRISE = 2,
  AFALL = 4,
  BFALL = 8,
};

volatile uint8_t last_left_sig;
volatile uint8_t last_right_sig;

volatile uint8_t left_moving_forward = 1;
volatile uint8_t right_moving_forward = 1;

void encoder_callback(uint gpio, uint32_t events)
{
  if (events & GPIO_IRQ_EDGE_RISE)
  {
    switch(gpio)
    {
        case RIGHT_ENCODER_A_PIN:
          right_moving_forward = last_right_sig & BRISE;
          last_right_sig = ARISE; 
          right_count++;
          break;
        case RIGHT_ENCODER_B_PIN:
          right_moving_forward = last_right_sig & AFALL;
          last_right_sig = BRISE;
          right_count++;
          break;
        case LEFT_ENCODER_A_PIN:
          left_moving_forward = last_left_sig & BFALL;
          last_left_sig = ARISE;
          left_count++;
          break;
        case LEFT_ENCODER_B_PIN:
          left_moving_forward = last_left_sig & ARISE;
          last_left_sig = BRISE;
          left_count++;
          break;
    }
  } else if (events & GPIO_IRQ_EDGE_FALL) {
    switch(gpio)
    {
        case RIGHT_ENCODER_A_PIN:
          right_moving_forward = last_right_sig & BFALL;
          last_right_sig = AFALL; 
          right_count++;
          break;
        case RIGHT_ENCODER_B_PIN:
          right_moving_forward = last_right_sig & ARISE;
          last_right_sig = BFALL;
          right_count++;
          break;
        case LEFT_ENCODER_A_PIN:
          left_moving_forward = last_left_sig & BRISE;
          last_left_sig = AFALL;
          left_count++;
          break;
        case LEFT_ENCODER_B_PIN:
          left_moving_forward = last_left_sig & AFALL;
          last_left_sig = BFALL;
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
    return ((float) count) / COUNTS_PER_REV * WHEEL_CIRCUMFERENCE_MM * CALIB;
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
  rpm_r = right_count / COUNTS_PER_REV * (SECONDS_PER_MIN/measure_period);
  rpm_l = left_count / COUNTS_PER_REV * (SECONDS_PER_MIN/measure_period);

  printf("rpm: %d %d \n", rpm_r, rpm_l);
  printf("counts: %ld %ld \n", right_count, left_count);
  printf("DISTS: %.1fmm %.1fmm \n", dist_r, dist_l);
  printf("left moving %s right moving %s\n", left_moving_forward ? "forward" : "backward", right_moving_forward ? "forward" : "backward");

  encoders_zero_distances();
  return true; // keep repeating
}