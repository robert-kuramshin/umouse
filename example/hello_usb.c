/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "hardware/flash.h"

#include "VL53L1X_api.h"
#include "VL53L1X_types.h"
#include <math.h>

#include "encoders.h"
#include "map.h"

#define RIGHT_MOTOR_A_PIN (14)
#define RIGHT_MOTOR_B_PIN (15)

#define LEFT_MOTOR_A_PIN (1)
#define LEFT_MOTOR_B_PIN (0)

#define WRAP (50000)

#define I2C_DEV_ADDR 0x29

#define RIGHT_ADC_PIN (26)
#define RIGHT_ADC_INDEX (0)

#define LEFT_ADC_PIN (27)
#define LEFT_ADC_INDEX (1)

#define PATH_LENGTH (10000)

char q[PATH_LENGTH];
int q_index = 0;

float right_dists[5];
float left_dists[5];
int ir_dist_index = 0;

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

void updateOdom()
{
  float dist = encoders_distance_traveled();
  mouseUpdateOdom(dist);
  encoders_zero_distances();
  printMaze();
}

void center_logic(uint16_t front_dist, float right_dist, float left_dist)
{
  int duty = 25;
  float tolerance = 0.2;
  float delta = right_dist - left_dist;
  // printf("Delta:%f\n", delta);
  if (front_dist <= 100) // this is what is causing the jittery continuous turns
  {
    brake(100);
    moving = false;
    updateOdom();
    if (right_dist >= 2) {
      // add a right gap to wall
      mouseUpdateWall(-1,DRIGHT);
    } else {
      mouseUpdateWall(1,DRIGHT);
    }
    if (left_dist >= 2) {
      // add a left gap to wall
      mouseUpdateWall(-1,DLEFT);
    } else {
      mouseUpdateWall(1,DLEFT);
    }
    mouseUpdateWall(1, DFORWARD);
  }
  else
  {
    moving = true;
    if (left_dist > 1.6 & right_dist > 1.6)
    {
      // we are far enough from the walls to assume were centered;
      goForward(duty);
    }
    if (left_dist < 1.6)
    {
      // veer right;
      moveRightMotor(32, 1);
      moveLeftMotor(23, 1);
    }
    else if (right_dist < 1.6)
    {
      // veer left
      moveRightMotor(23, 1);
      moveLeftMotor(32, 1);
    }
    else
    {
      // do nothing, were centered;
      goForward(duty);
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

void moveRightMotor(int duty, int direction)
{
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
  else if (direction == 0)
  {
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
  else if (direction == 0)
  {
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

void brake(int duty)
{
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

void core1_entry()
{
  // init tof sensor
  VL53L1X_Result_t results;
  VL53L1X_Status_t status = tof_init();
  printf("Done configuring sensor!\n");
  bool first_range = true;
  while (true)
  {
    // Wait until we have new data (front distance)
    uint8_t dataReady;
    do
    {
      status = VL53L1X_CheckForDataReady(I2C_DEV_ADDR, &dataReady);
      sleep_us(1);
    } while (dataReady == 0);

    // Read and display result
    status += VL53L1X_GetResult(I2C_DEV_ADDR, &results);

    if (results.distance > 10)
    {
      tof_distance = results.distance;
    }
    // printf("Core 1 TOF distance: %5d\n", tof_distance);

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

  multicore_launch_core1(core1_entry);

  // // Init motor output right and left (both forward)
  initMotors();
  gpio_pull_up(RIGHT_ENCODER_A_PIN);
  gpio_pull_up(RIGHT_ENCODER_B_PIN);
  gpio_pull_up(LEFT_ENCODER_A_PIN);
  gpio_pull_up(LEFT_ENCODER_B_PIN);

  encoders_register_callbacks();
  
  // gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_b);
  // // basic_sequence_test();
  // printf("Freq %d\n", 125000000/WRAP);

  // Init interrupts
  // gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_LEVEL_HIGH, true, &gpio_callback_a);

  // Repeating timer
  float prev_right_avg = 0.0f;
  float prev_left_avg = 0.0f;
  q[0] = 'R'; // start off going R
  // q[1] = 'R';
  //  q[2] = 'R';
  //  q[3] = 'R';
  //  q[4] = 'L';
  q_index = 0;
  while (true)
  {
    // if (ir_dist_index >= 5)
    // {
    //   for (int i = 0; i < 5; i++)
    //   {
    //     prev_right_avg += right_dists[i];
    //     prev_left_avg += left_dists[i];
    //   }
    //   prev_right_avg /= 5;
    //   prev_left_avg /= 5;
    // }
    // update ir distance arrays with new distances
    right_dists[ir_dist_index % 5] = rightVoltage2Distance(adcData((RIGHT_ADC_INDEX)));
    left_dists[ir_dist_index % 5] = leftVoltage2Distance(adcData(LEFT_ADC_INDEX));
    ir_dist_index++;

    // if (ir_dist_index > 5)
    // {
    //   float right_avg = 0.0f;
    //   float left_avg = 0.0f;
    //   for (int i = 0; i < 5; i++)
    //   {
    //     right_avg += right_dists[i];
    //     left_avg += left_dists[i];
    //   }
    //   right_avg /= 5;
    //   left_avg /= 5;
    //   updateOdom();
    //   if (right_avg - prev_right_avg > 0.1)
    //   {
    //     // add gap to right side of the map
    //     // mouseUpdateWall();
    //   }
    //   else
    //   {
    //     mouseUpdateWall(1,DRIGHT);
    //     // add a right wall to map
    //   }
    //   if (left_avg - prev_left_avg > 0.1)
    //   {
    //     // add gap to left side in ma[p]
    //   }
    //   else
    //   {
    //     mouseUpdateWall(1,DLEFT);
    //     // add a left wall to map
    //   }
    // }
    // 80mm ~ pi inches
    // printf("Core 2 TOF distance: %5d\n", tof_distance);
    // printf(" dist = %5d, rightDist = %f, leftDist = %f\n", tof_distance, right_dists[(ir_dist_index - 1) % 5], left_dists[(ir_dist_index - 1) % 5]);
    center_logic(tof_distance, right_dists[(ir_dist_index - 1) % 5], left_dists[(ir_dist_index - 1) % 5]);
    if (!moving)
    {
      sleep_ms(1000);
      // hardcoded directions for now to test the right motor, very unstable.
      if (true)
      {
        char next_action = q[q_index];
        if (next_action == 'R')
        {
          // this needs to use the encoder code (ie turn right for k cm)
          turn_right(40);
          sleep_ms(200);
          brake(100);
          mouseUpdateDir(DRIGHT);
        }
        else if (next_action == 'L')
        {
          // this needs to use the encoder code (ie turn right for k cm)
          turn_left(50);
          sleep_ms(300);
          brake(100);
          mouseUpdateDir(DLEFT);
        }
        else if (next_action == 'T')
        {
          // this needs to use the encoder code (ie turn right for k cm)
          turn_right(30);
          sleep_ms(600);
          brake(100);
          mouseUpdateDir(DRIGHT);
          mouseUpdateDir(DRIGHT);
        }
      }
      // zero encoders after turn
      encoders_zero_distances();
    }
  }
}
