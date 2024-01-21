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

#define I2C_DEV_ADDR_0 0x29
#define I2C_DEV_ADDR_1 0x39
#define I2C_DEV_ADDR_2 0x49

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

volatile uint16_t tof_distance[3] = {50,50,50};

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


VL53L1X_Status_t tof_init(int xshut, uint16_t addr)
{
  VL53L1X_Status_t status;
  // GPIO sda=16, scl=17, i2c0
  // gpio_pull_up(xshut);
  gpio_put(xshut,1);

  if (VL53L1X_I2C_Init(I2C_DEV_ADDR_0, i2c0) < 0)
  {
    printf("Error initializing sensor.\n");
    return 0;
  }
  printf("we are done\n");
  // change address
  if (addr != I2C_DEV_ADDR_0)
  {
    int ret = VL53L1X_SetI2CAddress(I2C_DEV_ADDR_0, addr<<1);
    if (ret != 0)
    {
      printf("Could not set sensor addr to %d\n", addr);
    } else {
      printf("address changed\n");
    }
  }
  // Ensure the sensor has booted
  uint8_t sensorState;
  do
  {
    status += VL53L1X_BootState(addr, &sensorState);
    VL53L1X_WaitMs(addr, 2);
  } while (sensorState == 0);
  printf("Sensor booted. 0x%x\n",addr);

  // Initialize and configure sensor
  status = VL53L1X_SensorInit(addr);
  status += VL53L1X_SetDistanceMode(addr, 1);
  status += VL53L1X_SetTimingBudgetInMs(addr, 100);
  status += VL53L1X_SetInterMeasurementInMs(addr, 100);
  status += VL53L1X_StartRanging(addr);

  return status;
}

void updateOdom()
{
  float dist = encoders_distance_traveled();
  mouseUpdateOdom(dist);
  encoders_zero_distances();
  // printMaze();
}

void center_logic(uint16_t front_dist, uint16_t right_dist, uint16_t left_dist)
{
  int duty = 25;
  if (front_dist < 100)
  {
    mouseUpdateWall(1, DFORWARD);
    brake(100);
    moving = false;
    updateOdom();
    if (right_dist < 100) {
      // add a right wall
      mouseUpdateWall(1,DRIGHT);
    } else {
      mouseUpdateWall(-1,DRIGHT);
    }
    if (left_dist < 100) {
      // add a left wall
      mouseUpdateWall(1,DLEFT);
    } else {
      mouseUpdateWall(-1,DLEFT);
    }
  }
  else
  {
    moving = true;
    if (left_dist > 50 & right_dist > 50)
    {
      // we are far enough from the walls to assume were centered;
      goForward(duty);
    }
    if (left_dist < 50)
    {
      // veer right;
      moveRightMotor(32, 1);
      moveLeftMotor(23, 1);
    }
    else if (right_dist < 50)
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

void core1_entry()
{
  // init tof sensor
  VL53L1X_Result_t results[3] = {0};
  VL53L1X_Status_t status[3] = {0};
  i2c_init(i2c0, VL53L1X_I2C_BAUDRATE);
  gpio_set_function(16, GPIO_FUNC_I2C);
  gpio_set_function(17, GPIO_FUNC_I2C);
  gpio_pull_up(16);
  gpio_pull_up(17);
  gpio_init(22);
  gpio_set_dir(22,GPIO_OUT);
  gpio_put(22,0);
  gpio_init(18);
  gpio_set_dir(18,GPIO_OUT);
  gpio_put(18,0);
  gpio_init(28);
  gpio_set_dir(28,GPIO_OUT);
  gpio_put(28,0);
  uint16_t addrs[3] ={I2C_DEV_ADDR_2, I2C_DEV_ADDR_1, I2C_DEV_ADDR_0};
  status[0] = tof_init(22,I2C_DEV_ADDR_2);
  printf("TOF 0 status %d\n",status[0]);
  status[1] = tof_init(18,I2C_DEV_ADDR_1);
  printf("TOF 1 status %d\n",status[1]);
  status[2] = tof_init(28,I2C_DEV_ADDR_0);
  printf("TOF 2 status %d\n",status[2]);
  printf("Done configuring sensor!\n");
  bool first_range[3] = {true,true,true};
  // this polling loop runs forever in thread 2
  while (true)
  {
    for(int i = 0; i < 3;i++)
    {
      // Wait until we have new data (front distance)
      uint8_t dataReady;
      do
      {
        status[i] = VL53L1X_CheckForDataReady(addrs[i], &dataReady);
        sleep_us(1);
      } while (dataReady == 0);

      // Read and display result
      status[i] += VL53L1X_GetResult(addrs[i], &results[i]);

      if (results[i].distance > 10)
      {
        tof_distance[i] = results[i].distance;
      }
      // printf("Core 1 TOF %d distance: %5d\n",i,  tof_distance[i]);
      status[i] += VL53L1X_ClearInterrupt(addrs[i]);
      // sleep_ms(1000);
      if (first_range[i])
      { // Clear twice on first measurement
        status[i] += VL53L1X_ClearInterrupt(addrs[i]);
        first_range[i] = false;
      }
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
  printf("Board initialized!\n");

  multicore_launch_core1(core1_entry);

  // Init motor output right and left (both forward)
  initMotors();
  gpio_pull_up(RIGHT_ENCODER_A_PIN);
  gpio_pull_up(RIGHT_ENCODER_B_PIN);
  gpio_pull_up(LEFT_ENCODER_A_PIN);
  gpio_pull_up(LEFT_ENCODER_B_PIN);
  encoders_register_callbacks();

  q[0] = 'R'; // start off going R
  q_index = 0;
  while (true)
  {
    uint16_t right_dist = tof_distance[1];
    uint16_t left_dist = tof_distance[2];
    updateOdom();
    if (right_dist < 150) {
      mouseUpdateWall(1,DRIGHT);
    } else {
      mouseUpdateWall(-1, DRIGHT);
    }

    if (left_dist < 150) {
      mouseUpdateWall(1, DLEFT);
    } else {
      mouseUpdateWall(-1, DLEFT);
    }
    // 80mm ~ pi inches
    // printf("Core 2 TOF distance: %5d\n", tof_distance[0], tof_distance[1], tof_distance[2]);
    // printf(" dist = %5d, rightDist = %f, leftDist = %f\n", tof_distance, right_dists[(ir_dist_index - 1) % 5], left_dists[(ir_dist_index - 1) % 5]);
    center_logic(tof_distance[0], right_dist, left_dist);
    if (!moving)
    {
      sleep_ms(1000);
      // hardcoded directions for now to test the right motor, very unstable.
      if (true)
      {
        // prompt left or right
        //printMaze();
        //printf("Go Left (L) or Right (R) ?\n");
        //char next_action = getchar_timeout_us(10 *1000*1000);
        char next_action = q[q_index];
        if (next_action == 'R')
        {
          printf("Going right\n");
          // this needs to use the encoder code (ie turn right for k cm)
          turn_right(30);
          sleep_ms(300);
          brake(100);
          mouseUpdateDir(DRIGHT);
        }
        else if (next_action == 'L')
        {
          printf("Going left\n");
          // this needs to use the encoder code (ie turn right for k cm)
          turn_left(50);
          sleep_ms(300);
          brake(100);
          mouseUpdateDir(DLEFT);
        }
        else if (next_action == 'T')
        {
          printf("turning around\n");
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