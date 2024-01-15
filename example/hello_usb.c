/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"

#include "VL53L1X_api.h"
#include "VL53L1X_types.h"
#include <math.h>

#define RIGHT_MOTOR_A_PIN (14)
#define RIGHT_MOTOR_B_PIN (15)
#define RIGHT_ENCODER_A_PIN (12)
#define RIGHT_ENCODER_B_PIN (13)

#define LEFT_MOTOR_A_PIN (1)
#define LEFT_MOTOR_B_PIN (0)
#define LEFT_ENCODER_A_PIN (2)
#define LEFT_ENCODER_B_PIN (3)

#define WRAP (50000)

#define COUNTS_PER_REV (12)
#define SECONDS_PER_MIN (60)
#define MEASURE_PERIOD_SEC (3)
#define SEC_TO_uS (1000000)

#define I2C_DEV_ADDR 0x29

#define RIGHT_ADC_PIN (26)
#define RIGHT_ADC_INDEX (0)

#define LEFT_ADC_PIN (27)
#define LEFT_ADC_INDEX (1)

// For the TOF, we are using : sda -> gpio16 scl -> gpio17 xshut -> pulled high at gpio22

volatile uint32_t counter = 0;

volatile uint16_t tof_distance = 50;

bool moving = true;
// motor slices and f/r channels
uint rslice;
uint rfchan;
uint rrchan;
uint lslice;
uint lfchan;
uint lrchan;

// void gpio_callback_a(uint gpio, uint32_t events) {
//     printf("event A\n");
// }

void gpio_callback_b(uint gpio, uint32_t events)
{
  // printf("event B\n");
  if (events & GPIO_IRQ_EDGE_FALL)
  {
    counter += 1;
  }
}

bool timer_callback(repeating_timer_t *rt)
{
  uint32_t count = counter;
  counter = 0;

  uint32_t rpm = (count * (SECONDS_PER_MIN / MEASURE_PERIOD_SEC)) / COUNTS_PER_REV;

  printf("RPM: %d COUNT: %d \n", rpm, count);
  return true; // keep repeating
}

VL53L1X_Status_t tof_init()
{
  VL53L1X_Status_t status;
  // GPIO sda=16, scl=17, i2c0
  if (VL53L1X_I2C_Init(I2C_DEV_ADDR, i2c0) < 0)
  {
    printf("Error initializing sensor.\n");
    return 0;
  }
  printf("we are done\n");
  // Ensure the sensor has booted
  uint8_t sensorState;
  do
  {
    status += VL53L1X_BootState(I2C_DEV_ADDR, &sensorState);
    VL53L1X_WaitMs(I2C_DEV_ADDR, 2);
  } while (sensorState == 0);
  printf("Sensor booted.\n");
  // Initialize and configure sensor
  status = VL53L1X_SensorInit(I2C_DEV_ADDR);
  status += VL53L1X_SetDistanceMode(I2C_DEV_ADDR, 1);
  status += VL53L1X_SetTimingBudgetInMs(I2C_DEV_ADDR, 100);
  status += VL53L1X_SetInterMeasurementInMs(I2C_DEV_ADDR, 100);
  status += VL53L1X_StartRanging(I2C_DEV_ADDR);

  return status;
}

// 12-bit conversion, assume max value == ADC_VREF == 3.3 V
float adcData(int adc_index)
{
  adc_select_input(adc_index);
  float voltage = adc_read() * (3.3f / (1 << 12));
  return voltage;
}

float rightVoltage2Distance(float data)
{
  // interpolate later, for now get crude dists
  float m;
  if (0.095 < data && data < 0.2)
  {
    m = data - 0.095;
    return 0.5 - m * 4.76;
  }
  else if (0.062 < data && data < 0.095)
  {
    m = data - 0.062;
    return 1.0 - m * 15.15;
  }
  else if (0.045 < data && data < 0.062)
  {
    m = data - 0.045;
    return 1.5 - m * 29.41;
  }
  return 2.0;
}
float leftVoltage2Distance(float data)
{
  float m;
  if (0.05 < data && data < 0.07)
  {
    m = data - 0.05;
    return 0.5 - m * 25;
  }
  else if (0.032 < data && data < 0.05)
  {
    m = data - 0.032;
    return 1.0 - m * 27.77;
  }
  else if (0.0265 < data && data < 0.032)
  {
    m = data - 0.0265;
    return 1.5 - m * 90.9;
  }
  return 2.0;
}

void center_logic(uint16_t front_dist, float right_dist, float left_dist)
{
  // take distance measurements and send motor commands to keep center
  // lets say right and left distance max is 1.5 inches
  int duty = 25;
  int turnmag = 7;
  float tolerance = 0.2;
  float delta = right_dist - left_dist;
  // printf("Delta:%f\n", delta);
  int sign = 0;
  if (left_dist < 1.9 & right_dist < 1.9) {
    sign = 2;
  }
  if (left_dist < 1.9) {
    sign = 1;
  } else if (right_dist < 1.9) {
    sign = -1;
  } else {
    sign = 0;
  }
  
  if (front_dist <= 50) {
    // attempt a right turn;
    // if (left_dist == 2.0 && right_dist < 2.0){
    //   float curr_dist = right_dist;
    //   moveRightMotor(40, 1);
    //   moveLeftMotor(40, -1);
    //   while (curr_dist <= right_dist + 0.2 || curr_dist >= right_dist - 0.2){
    //     right_dist = rightVoltage2Distance(adcData(RIGHT_ADC_PIN));
    //   }
    //   // done turn, stop the vehicle;
    //   // goForward(0);
    // }
    brake(100);
    moving = false;
  }
   else {
    if (!moving) {
      goForward(100);
      sleep_ms(100);
      //sleep_ms(150);
    }
    moving = true;
    if (sign == 0 || sign == 2)
  {
    // do nothing, we're centered
    goForward(duty);
  }
  // if delta > tol and sign == -1 -> increase duty in right wheel
  else if (sign == -1)
  {
    moveRightMotor(32, 1);
    moveLeftMotor(23, 1);
    // moving = false;
  }
  else if (sign == 1)
  {
    moveRightMotor(23, 1);
    moveLeftMotor(32, 1);
    // moving = false;
  }
   }
}


// 0 <= duty <= 100
void pwm_set_duty(uint slice_num, uint chan, int duty)
{
  pwm_set_chan_level(slice_num, chan, duty * WRAP / 100);
}

void initMotors()
{
  gpio_set_function(RIGHT_MOTOR_A_PIN, GPIO_FUNC_PWM);
  rslice = pwm_gpio_to_slice_num(RIGHT_MOTOR_A_PIN);
  rfchan = pwm_gpio_to_channel(RIGHT_MOTOR_A_PIN);
  gpio_set_function(RIGHT_MOTOR_B_PIN, GPIO_FUNC_PWM);
  rrchan = pwm_gpio_to_channel(RIGHT_MOTOR_B_PIN);
  pwm_set_enabled(rslice, true);
  pwm_set_wrap(rslice, WRAP);
  // left motor init
  gpio_set_function(LEFT_MOTOR_A_PIN, GPIO_FUNC_PWM);
  lslice = pwm_gpio_to_slice_num(LEFT_MOTOR_A_PIN);
  lfchan = pwm_gpio_to_channel(LEFT_MOTOR_A_PIN);
  gpio_set_function(LEFT_MOTOR_B_PIN, GPIO_FUNC_PWM);
  lrchan = pwm_gpio_to_channel(LEFT_MOTOR_B_PIN);
  pwm_set_enabled(lslice, true);
  pwm_set_wrap(lslice, WRAP);
}

void moveRightMotor(int duty, int direction) {
  // forward is dir = 1
    if (direction == 1)
    {
      pwm_set_duty(rslice, rrchan, 0);
      pwm_set_duty(rslice, rfchan, duty);
    }
    else if (direction == -1)
    {
      pwm_set_duty(rslice, rrchan, duty);
      pwm_set_duty(rslice, rfchan, 0);
    }
     else if (direction == 0) {
      pwm_set_duty(rslice, rrchan, duty);
      pwm_set_duty(rslice, rfchan, duty);
     }

    else
    {
      // neutral?
      pwm_set_duty(rslice, rrchan, 0);
      pwm_set_duty(rslice, rfchan, 0);
    }
}

void moveLeftMotor(int duty, int direction)
{
    // forward is dir = 1
    if (direction == 1)
    {
      pwm_set_duty(lslice, lrchan, 0);
      pwm_set_duty(lslice, lfchan, duty);
    }
    else if (direction == -1)
    {
      pwm_set_duty(lslice, lrchan, duty);
      pwm_set_duty(lslice, lfchan, 0);
    }
    else if (direction == 0) {
      pwm_set_duty(lslice, lrchan, duty);
      pwm_set_duty(lslice, lfchan, duty);
     }
    else
    {
      // neutral?
      pwm_set_duty(lslice, lrchan, 0);
      pwm_set_duty(lslice, lfchan, 0);
    }
}

void brake(int duty){
  moveRightMotor(duty, 0);
  moveLeftMotor(duty, 0);
}

void goForward(int duty)
{
  moveRightMotor(duty, 1);
  moveLeftMotor(duty, 1);
}

void goBackward(int duty)
{
  moveRightMotor(duty, -1);
  moveLeftMotor(duty, -1);
}

void turn_left(int duty)
{
  moveRightMotor(duty, 1);
  moveLeftMotor(duty, -1);
}

void turn_right(int duty)
{
  moveRightMotor(duty, -1);
  moveLeftMotor(duty, 1);
}

void counter_total_zero_test()
{
  gpio_set_function(RIGHT_MOTOR_A_PIN, GPIO_FUNC_PWM);
  gpio_set_function(RIGHT_MOTOR_B_PIN, GPIO_FUNC_PWM);
  gpio_set_function(LEFT_MOTOR_A_PIN, GPIO_FUNC_PWM);
  gpio_set_function(LEFT_MOTOR_B_PIN, GPIO_FUNC_PWM);

  sleep_us(5 * 1000 * 1000);

  goForward(50);
  sleep_us(2 * 1000 * 1000);

  goForward(0);
  sleep_us(3 * 1000 * 1000);

  goBackward(50);
  sleep_us(2 * 1000 * 1000);

  goBackward(0);
}

void core1_entry() {
  // init tof sensor
  VL53L1X_Result_t results;
  VL53L1X_Status_t status = tof_init();
  printf("Done configuring sensor!\n");
  bool first_range = true;
  while (true) {
    // Wait until we have new data (front distance)
    uint8_t dataReady;
    do
    {
      status = VL53L1X_CheckForDataReady(I2C_DEV_ADDR, &dataReady);
      sleep_us(1);
    } while (dataReady == 0);

     // Read and display result
    status += VL53L1X_GetResult(I2C_DEV_ADDR, &results);

    if (results.distance > 10) {
      tof_distance = results.distance;
    }
    printf("Core 1 TOF distance: %5d\n", tof_distance);

    status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
    if (first_range)
    { // Clear twice on first measurement
      status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
      first_range = false;
    }
}
}

int main()
{
  stdio_init_all();
  adc_init();

  if (cyw43_arch_init())
  {
    printf("Wi-Fi init failed");
    return -1;
  }
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  sleep_ms(1000);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  sleep_ms(1000);
  printf("Board initialized!");

  // Initialize adc pins
  adc_gpio_init(RIGHT_ADC_PIN);
  adc_gpio_init(LEFT_ADC_PIN);
  
  // init tof sensor
  // VL53L1X_Result_t results;
  // VL53L1X_Status_t status = tof_init();
  // printf("Done configuring sensor!\n");

  multicore_launch_core1(core1_entry);


  // // Init motor output right and left (both forward)
  initMotors();
  gpio_pull_up(RIGHT_ENCODER_A_PIN);
  gpio_pull_up(RIGHT_ENCODER_B_PIN);
  gpio_pull_up(LEFT_ENCODER_A_PIN);
  gpio_pull_up(LEFT_ENCODER_B_PIN);
  
  // gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_b);
  // // basic_sequence_test();
  // printf("Freq %d\n", 125000000/WRAP);

  // Init interrupts
  // gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_LEVEL_HIGH, true, &gpio_callback_a);

  // Repeating timer
  repeating_timer_t timer;
  add_repeating_timer_us(-1 * MEASURE_PERIOD_SEC * SEC_TO_uS, timer_callback, NULL, &timer);
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
  //     goForward(level);
  // }
  bool first_range = true;
  int min_distance = 50;
  int count = 1;
  while (true)
  {
    // Get right distance
    float right_dist = rightVoltage2Distance(adcData((RIGHT_ADC_INDEX)));

    // Get left distance
    float left_dist = leftVoltage2Distance(adcData(LEFT_ADC_INDEX));

    //printf(" dist = %5d, rightDist = %f, leftDist = %f\n",tof_distance, right_dist, left_dist);

    // 80mm ~ pi inches
    printf("Core 2 TOF distance: %5d\n", tof_distance);
    center_logic(tof_distance, right_dist, left_dist);
    // sleep_ms(1000);
  }
}
