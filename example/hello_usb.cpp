/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/mutex.h"
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

#include "logflash.h"

#include "pico/flash.h"

#include "encoders.h"
#include "map.h"
#include "engine.h"

#include "pico/float.h"

#define PICO_DEFAULT_LED_PIN (25)

#define RIGHT_MOTOR_A_PIN (15)
#define RIGHT_MOTOR_B_PIN (14)

#define LEFT_MOTOR_A_PIN (0)
#define LEFT_MOTOR_B_PIN (1)

#define WRAP (50000)

#define I2C_DEV_ADDR_0 0x29
#define I2C_DEV_ADDR_1 0x39
#define I2C_DEV_ADDR_2 0x49

#define RIGHT_ADC_PIN (26)
#define RIGHT_ADC_INDEX (0)

#define LEFT_ADC_PIN (27)
#define LEFT_ADC_INDEX (1)

#define PATH_LENGTH (10000)

#define PI (3.14159)

#define RED_PIN (8)
#define YELLOW_PIN (7)
#define GREEN_PIN (6)

#define BUTTON_PIN (9)

#define COORD_TO_NUM(x,y) ((x) * MAZE_WIDTH + (y))
#define min(a,b) (((a)<(b))?(a):(b))

int target_states[1] = {COORD_TO_NUM(2,0)};

// For the TOF, we are using : sda -> gpio16 scl -> gpio17 xshut -> pulled high at gpio22

volatile uint32_t counter = 0;

volatile bool sensors_ready = false;

volatile uint16_t tof_distance[3] = { 151, 42, 42 };  // make sure not to do anything till we get a first reading

bool moving = false;
// motor slices and f/r channels
uint rslice;
uint rfchan;
uint rrchan;
uint lslice;
uint lfchan;
uint lrchan;
int break_dist = 12;
void smart_stop();
void moveLeftMotor(int duty, int direction);
void moveRightMotor(int duty, int direction);
void goForward(int duty);


volatile float angle = 0;
mutex_t m;
float angle_offset = 0;

void initLEDS()
{
  gpio_init(GREEN_PIN);
  gpio_init(RED_PIN);
  gpio_init(YELLOW_PIN);

  gpio_set_dir(GREEN_PIN, GPIO_OUT);
  gpio_set_dir(RED_PIN, GPIO_OUT);
  gpio_set_dir(YELLOW_PIN, GPIO_OUT);
}
void setGreenStatus()
{
  gpio_put(GREEN_PIN,1);
  gpio_put(RED_PIN,0);
  gpio_put(YELLOW_PIN,0);
}

void setRedStatus()
{
  gpio_put(GREEN_PIN,0);
  gpio_put(RED_PIN,1);
  gpio_put(YELLOW_PIN,0);
}

void setYellowStatus()
{
  gpio_put(GREEN_PIN,0);
  gpio_put(RED_PIN,0);
  gpio_put(YELLOW_PIN,1);
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

// void gpio_callback_a(uint gpio, uint32_t events) {
//     printf("event A\n");
// }

VL53L1X_Status_t tof_init(int xshut, uint16_t addr)
{
  VL53L1X_Status_t status;
  // GPIO sda=16, scl=17, i2c0
  // gpio_pull_up(xshut);
  gpio_put(xshut, 1);

  if (VL53L1X_I2C_Init(I2C_DEV_ADDR_0, i2c0) < 0)
  {
    printf("Error initializing sensor.\n");
    return 0;
  }
  printf("we are done\n");
  // change address
  if (addr != I2C_DEV_ADDR_0)
  {
    int ret = VL53L1X_SetI2CAddress(I2C_DEV_ADDR_0, addr << 1);
    if (ret != 0)
    {
      printf("Could not set sensor addr to %d\n", addr);
    }
    else
    {
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
  printf("Sensor booted. 0x%x\n", addr);

  // Initialize and configure sensor
  status = VL53L1X_SensorInit(addr);
  status += VL53L1X_SetDistanceMode(addr, 1);
  status += VL53L1X_SetTimingBudgetInMs(addr, 100);
  status += VL53L1X_SetInterMeasurementInMs(addr, 100);
  status += VL53L1X_StartRanging(addr);

  return status;
}

float last_odom = 0;

void zeroOdom()
{
  last_odom = 0;
  encoders_zero_distances();
}
void updateOdom()
{
  float dist = encoders_distance_traveled();
  mouseUpdateOdom(dist - last_odom);
  last_odom = dist;
}


// jittery vals
// #define Kp (0.07)
// #define Kd (-0.1)
// #define Kpa (15)
// #define Kda (1.0)
// #define DEFAULT_DUTY (26)
// #define GOALDIST (55)


// crack vals
#define Kp (0.1)
#define Kd (-0.2)
#define Kpa (10.0)

#define DEFAULT_DUTY (28)
#define GOALDIST (42)

int curr_left_duty = DEFAULT_DUTY;
int curr_right_duty = DEFAULT_DUTY;
float last_right_dist = GOALDIST;
float last_left_dist = GOALDIST;
void center_logic(uint16_t front_dist, uint16_t right_dist, uint16_t left_dist, float desired_heading)
{
  if (tof_distance[0] >  80)
  {
    moving = true;
    float heading;
    if(desired_heading == 0)
    {
      heading = min(abs(360-angle),abs(0-angle));
    } else {
      heading = abs(desired_heading-angle);
    }
    float angle_error;
    if(desired_heading == 0)
    {
      angle_error = sin(angle*PI/180);
    }  else if (desired_heading == 180)
    {
      angle_error = -1 * sin(angle*PI/180);
    } else if (desired_heading == 90) {
      angle_error = -1*cos(angle*PI/180);
    } else if (desired_heading == 270)
    {
      angle_error = cos(angle*PI/180);
    }

    float right = (GOALDIST - cos(heading* PI/180)*right_dist);
    float ar = right*Kp + (last_right_dist - right) * Kd + angle_error*Kpa;
    if (abs(GOALDIST-right_dist) > GOALDIST)
    {
      curr_right_duty = DEFAULT_DUTY;
    } else {
      curr_right_duty = DEFAULT_DUTY + (int)ar;
    }
    last_right_dist = right;
    
    float left = (GOALDIST - cos(heading* PI/180)*left_dist);
    float al = left*Kp + (last_left_dist - left) * Kd - angle_error*Kpa;
    if (abs(GOALDIST-left_dist) > GOALDIST)
    {
      curr_left_duty = DEFAULT_DUTY;
    } else {
      curr_left_duty = DEFAULT_DUTY + (int)al;
    }
    last_left_dist = left;
    // printf("h: %f rl: %f rr: %f  l: %d r: %d\n",heading, cos(heading* PI/180)*left_dist,cos(heading* PI/180)*right_dist,  left_dist, right_dist);
    // sleep_ms(100);
    // printf("cr %d, cl %d\n", curr_right_duty, curr_left_duty);
    moveLeftMotor(curr_left_duty,1);
    moveRightMotor(curr_right_duty,1);
  } else {
    smart_stop();
    printMaze();
    mouseUpdateWall(3, DFORWARD);
    moving = false;
    if (right_dist < 100)
    {
      // add a right wall
      mouseUpdateWall(3, DRIGHT);
    }
    else
    {
      mouseUpdateWall(-3, DRIGHT);
    }
    if (left_dist < 100)
    {
      // add a left wall
      mouseUpdateWall(3, DLEFT);
    }
    else
    {
      mouseUpdateWall(-3, DLEFT);
    }

    goBackward(30);
    while(tof_distance[0] < 30)
    {
      sleep_us(1);
    }
    smart_stop();
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

#define HMC5883L_ADDR 0x1E
#define CONFIG_REG_A 0x00
#define CONFIG_REG_B 0x01
#define MODE_REG 0x02
#define DATA_REG 0x03
#define LSB_TO_UT (100.0 / 1090.0)

void init_mpu()
{
  uint8_t data[2] = { CONFIG_REG_A, 0x78 };
  i2c_write_blocking(i2c0, HMC5883L_ADDR, data, 2, false);
  data[0] = CONFIG_REG_B;
  data[1] = 0x20;
  i2c_write_blocking(i2c0, HMC5883L_ADDR, data, 2, false);
  data[0] = MODE_REG;
  data[1] = 0x00;
  i2c_write_blocking(i2c0, HMC5883L_ADDR, data, 2, false);
}

static void read_registers(uint8_t reg, uint8_t* buf, uint16_t len)
{
  reg |= 1;
  i2c_write_blocking(i2c0, HMC5883L_ADDR, &reg, 1, false);
  i2c_read_blocking(i2c0, HMC5883L_ADDR, buf, len, false);
}


void core1_entry()
{
  flash_safe_execute_core_init();
  // init tof sensor
  VL53L1X_Result_t results[3] = { 0 };
  VL53L1X_Status_t status[3] = { 0 };

  i2c_init(i2c0, 100000);

  gpio_set_function(16, GPIO_FUNC_I2C);
  gpio_set_function(17, GPIO_FUNC_I2C);

  gpio_pull_up(16);
  gpio_pull_up(17);

  // mpu.calibrateAccelGyro();

  // mpu.setMagBias(-507.45,81.90,122.26);
  // mpu.setMagScale(0.92, 1.07, 1.02 );

  // AK8963 mag biases (mG)
  // -512.79, 23.15, 148.09
  // AK8963 mag scale (mG)
  // 0.91, 1.05, 1.06

  // mpu.setMagBias(-510.12,52.525, 135.175);
  // mpu.setMagScale(0.915, 1.06, 1.04 );

  init_mpu();

  gpio_init(22);
  gpio_set_dir(22, GPIO_OUT);
  gpio_put(22, 0);
  gpio_init(18);
  gpio_set_dir(18, GPIO_OUT);
  gpio_put(18, 0);
  gpio_init(28);
  gpio_set_dir(28, GPIO_OUT);
  gpio_put(28, 0);
  uint16_t addrs[3] = { I2C_DEV_ADDR_2, I2C_DEV_ADDR_1, I2C_DEV_ADDR_0 };
  status[0] = tof_init(22, I2C_DEV_ADDR_2);
  printf("TOF 0 status %d\n", status[0]);
  status[1] = tof_init(18, I2C_DEV_ADDR_1);
  printf("TOF 1 status %d\n", status[1]);
  status[2] = tof_init(28, I2C_DEV_ADDR_0);
  printf("TOF 2 status %d\n", status[2]);
  printf("Done configuring sensor!\n");
  sensors_ready = true;
  bool first_range[3] = { true, true, true };
  float x, y, z;
  float xl = 0, yl = 0, zl = 0;
  uint8_t rawdata[6] = { 0 };

  // this polling loop runs forever in thread 2
  while (true)
  {
    for (int i = 0; i < 4; i++)
    {
      if (i == 3)
      {
        // try reading mpu (nonblocking)
        read_registers(DATA_REG, rawdata, 6);
        x = (rawdata[0] << 8) | rawdata[1];
        z = (rawdata[2] << 8) | rawdata[3];
        y = (rawdata[4] << 8) | rawdata[5];
        if (x > 32767)
          x -= 65536;
        if (y > 32767)
          y -= 65536;
        if (z > 32767)
          z -= 65536;

        x *= LSB_TO_UT;
        y *= LSB_TO_UT;
        z *= LSB_TO_UT;

        // apply calibration
        x -= -28.027523;
        y -= -8.944954;
        z -= 11.972478;

        // smooth a bit
        // if (xl != 0)
        // {
        //   x = (x + xl * 4) / 5;
        //   y = (y + yl * 4) / 5;
        //   z = (z + zl * 4) / 5;
        //   xl = x;
        //   yl = y;
        //   zl = z;
        // }

        float k = atan2f(x, y);

        k = k * 180 / 3.14159265;
        if (k < 0)
        {
          k = k + 360;
        }
        if (angle_offset == 0)
        {
          angle_offset = 360 - (k);
          printf("Angle offset %f, angle %f\n", angle_offset, k);
        }
        // mutex_enter_blocking(&m);
        angle = fmod(k + angle_offset, 360);
        // printf("core 1: angle %f, k: %f of: %f\n", angle, k, angle_offset);
        // mutex_exit(&m);

        continue;
      }
      //
      // Wait until we have new data (front distance)
      uint8_t dataReady;
      status[i] = VL53L1X_CheckForDataReady(addrs[i], &dataReady);
      if (dataReady == 0)
      {
        // skip sensor if not ready
        continue;
      }

      // Read and display result
      status[i] += VL53L1X_GetResult(addrs[i], &results[i]);

      if (results[i].distance > 0)
      {
        tof_distance[i] = results[i].distance;
      }
      // printf("Core 1 TOF %d distance: %5d\n",i,  tof_distance[i]);
      status[i] += VL53L1X_ClearInterrupt(addrs[i]);
      // sleep_ms(1000);
      if (first_range[i])
      {  // Clear twice on first measurement
        status[i] += VL53L1X_ClearInterrupt(addrs[i]);
        first_range[i] = false;
      }
    }
  }
}

void performInstructions(char* instructions) {
  
}

#define THR_CNT (1)
#define THR_SLP (1)
void smart_stop()
{
  // first brake
  // brake(100);

  // record original counts and direction
  uint32_t last_right = right_count;
  uint32_t last_left = left_count;
  uint8_t left_orig_dir = left_moving_forward;
  uint8_t right_orig_dir = right_moving_forward;

  moveLeftMotor(100, left_moving_forward ? -1 : 1);
  moveRightMotor(100, right_moving_forward ? -1 : 1);

  bool run = true;
  while (run)
  {
    run = false;
    if ((left_orig_dir > 0) == (left_moving_forward > 0))
    {
      // if count change > threshold and direction hasn't changed
      run = true;
      // printf("hardbreak left %d \n", left_count - last_left);
      // apply full duty in opposite direction
      moveLeftMotor(100, left_orig_dir ? -1 : 1);
    }
    else
    {
      moveLeftMotor(100, 0);
    }
    if ((right_orig_dir > 0) == (right_moving_forward > 0))
    {
      // if count change > threshold and direction hasn't changed
      run = true;
      // printf("hardbreak right %d\n", right_count - last_right);
      // apply full duty in opposite direction
      moveRightMotor(100, right_orig_dir ? -1 : 1);
    }
    else
    {
      moveRightMotor(100, 0);
    }
  }
}

// void smartStart()
// {
//   // init some drive strength
//   // keep increasing power until encoders start reading
//   // drop drive speed
// }

void goLeft()
{
  int curr_quad;
  int low_quad = (int)(angle / 90);
  int high_quad = low_quad + 1;
  if (angle - 90 * low_quad > 90 * high_quad - angle)
  {
    curr_quad = high_quad;
  }
  else
  {
    curr_quad = low_quad;
  }
  int target_quad = (curr_quad + 3) % 4;
  float left_c = sin(angle * PI / 180);
  float right_c = sin((target_quad * 90 + break_dist) * PI / 180);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  turn_left(35);
  sleep_ms(50);
  if (target_quad == 0 || target_quad == 3)
  {
    while (left_c > right_c)
    {
      turn_left(28);
      // printf("angle %f, while %f > %f\n", angle, left_c, right_c);
      left_c = sin(angle * PI / 180);
      right_c = sin((target_quad * 90 + break_dist) * PI / 180);
    }
  }
  else
  {
    while (left_c < right_c)
    {
      turn_left(28);
      // printf("angle %f, while %f > %f\n", angle, left_c, right_c);
      left_c = sin(angle * PI / 180);
      right_c = sin((target_quad * 90 + break_dist) * PI / 180);
    }
  }
  printf("angle %f\n", angle);
  printf("done turning\n");
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  smart_stop();

  // zero without updating with data from turn
  zeroOdom();
}

void goRight() {
  int curr_quad;
  int low_quad = (int)(angle / 90);
  int high_quad = low_quad + 1;
  if (angle - 90 * low_quad > 90 * high_quad - angle)
  {
    curr_quad = high_quad;
  }
  else
  {
    curr_quad = low_quad;
  }
  
  int target_quad = (curr_quad % 4) + 1;
  float left_c = sin(angle * PI / 180);
  float right_c = sin((target_quad * 90 - break_dist) * PI / 180);

  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  turn_right(35);
  sleep_ms(50);
  if (target_quad == 4 || target_quad == 1)
  {
    while (left_c < right_c)
    {
      // printf("angle %f, while %f < %f\n", angle, left_c, right_c);
      left_c = sin(angle * PI / 180);
      right_c = sin((target_quad * 90 - break_dist) * PI / 180);
      turn_right(28);
    }
    printf("angle %f, while %f > %f\n", angle, left_c, right_c);
  }
  else
  {
    while (left_c > right_c)
    {
      // printf("angle %f, while %f > %f\n", angle, left_c, right_c);
      left_c = sin(angle * PI / 180);
      right_c = sin((target_quad * 90 - break_dist) * PI / 180);
      turn_right(28);
    }
    printf("angle %f, while %f > %f\n", angle, left_c, right_c);
  }
  printf("angle %f\n", angle);
  printf("done turning\n");
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  smart_stop();

  // zero without updating odom with data from turn
  zeroOdom();
}

int explorationRun() {
  lfprintf("Mouse starting....\n");
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  sleep_ms(1000);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  sleep_ms(1000);

  // wait for TOFs and gyro to init
  while (!sensors_ready || angle == 0)
  {
    sleep_ms(50);
  }
  int turn_around_counter = 0;

  int start_cell = -1;
  while (true)
  {
    updateOdom();
    state_t ms = mouseGetState();
    uint16_t right_dist = tof_distance[1];
    uint16_t left_dist = tof_distance[2];
    int curr_cell = ms.x * MAZE_HEIGHT + ms.y;
    for (int i = 0; i< sizeof(target_states)/sizeof(target_states[0]); i++)
    {
      if (COORD_TO_NUM(ms.x,ms.y) == target_states[i])
      {
        smart_stop();
        mouseUpdateWall(right_dist < 70 ? 1 : -1, DRIGHT);
        mouseUpdateWall(left_dist < 70 ? 1 : -1, DLEFT);
        int8_t *v = getVWalls();
        int8_t *h = getHWalls();
        write_walls(h,v);
        update_target(target_states[i]);
        setGreenStatus();
        return 0;
      }
    }
    // printMaze();
    // build map while we are moving
    if (start_cell != curr_cell){
        if (right_dist < 70)
      {
        mouseUpdateWall(1, DRIGHT);
      }
      else
      {
        mouseUpdateWall(-1, DRIGHT);
      }

      if (left_dist < 70)
      {
        mouseUpdateWall(1, DLEFT);
      }
      else
      {
        mouseUpdateWall(-1, DLEFT);
      }
    }
    // 80mm ~ pi inches
    // printf("Core 2 TOF distance: %5d\n", tof_distance[0], tof_distance[1], tof_distance[2]);
    // printf(" Front dist = %5d, rightDist = %5d\n, leftDist = %5d\n", tof_distance[0], tof_distance[1], tof_distance[2]);
    float desired_heading;
    if (ms.ori == ORIGHT)
    {
      desired_heading = 0;
    } else if(ms.ori == OUP)
    {
      desired_heading = 270;
    } else if(ms.ori == OLEFT)
    {
      desired_heading = 180;
    } else if(ms.ori == ODOWN)
    {
      desired_heading = 90;
    }

    center_logic(tof_distance[0], right_dist, left_dist, desired_heading);
    if (!moving)
    {
      sleep_ms(1000);
      printf("We have stopped moving!");
      updateOdom();
      // printf("Go Left (L) or Right (R) ?\n");
      // char next_action = getchar_timeout_us(10 *1000*1000);

      if (mouseCanGoRight() == 1 || tof_distance[1] > 150)
      {
        printf("Going right\n");
        printf("angle %f\n", angle);
        // int curr_quad = (int) (angle / 90);

        goRight();
        // gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(1000);
        // gpio_put(PICO_DEFAULT_LED_PIN, 0);
      }
      else if (mouseCanGoLeft() == 1 || tof_distance[2] > 150)
      {
        printf("Going left\n");
        goLeft();
        // gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(1000);
        // gpio_put(PICO_DEFAULT_LED_PIN, 0);
      }
      else
      {
        if (turn_around_counter > 10)
        {
          turn_around_counter = 0;
          printf("turning around\n");
          while (right_count < 160)
          {
            turn_left(35);
          }
          smart_stop();
          sleep_ms(1000);
          mouseUpdateDir(DBACKWARDS);
        }
        else
        {
          turn_around_counter++;
        }
      }

      // update orientation based on heading
      float dr = min(abs(0-angle),abs(160-angle));
      float dd = abs(90-angle);
      float dl = abs(180-angle);
      float du = abs(270-angle);
      float m = min(min(dr, dd), min(dl, du));
      if (m == dr)
      {
        mouseUpdateOri(ORIGHT);
      } else if (m == dd)
      {
        mouseUpdateOri(ODOWN);
      } else if (m == dl)
      {
        mouseUpdateOri(OLEFT);
      } else if (m == du)
      {
        mouseUpdateOri(OUP);
      }

      start_cell = ms.x * MAZE_HEIGHT + ms.y;;
      gpio_put(PICO_DEFAULT_LED_PIN, 0);
      last_right_dist = GOALDIST;
      last_left_dist = GOALDIST;
    }
  }
}

int solvedRun(char* instructions) {
  for (int i = 0; i < MAZE_HEIGHT * MAZE_WIDTH; i++){
    char instruction = instructions[i];
    if (instructions[i] == 0)
    {
      lfprintf("solver: done solving!\n");
      smart_stop();
      return 1;
    }
    state_t curr_state = mouseGetState();
    int start_cell = curr_state.x * MAZE_HEIGHT + curr_state.y;
    int curr_cell = start_cell;
    while (start_cell == curr_cell) {
      if (instruction == 'R') {
        // turn right
        lfprintf("solver: going right!\n");
        goRight();
      }
      if (instruction == 'L') {
        lfprintf("solver: going left!\n");
        goLeft();
        // turn left
      }
      if (instruction == 'B') {
        lfprintf("solver: turning 'round!\n");
        goRight();
        goRight();
        // turn right twice
      }
      instruction = 'S';
      goForward(35);
      updateOdom();
      curr_state = mouseGetState();
      curr_cell = curr_state.x * MAZE_HEIGHT + curr_state.y;
      // smart_stop();
      printf("x: %d y: %d cell: %d ori: %d \n",curr_state.x, curr_state.y, curr_cell, curr_state.ori);

    }
  }
}


int main()
{
  stdio_init_all();
  adc_init();

  initLEDS();

  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);
  gpio_pull_up(BUTTON_PIN);
  sleep_ms(1);
  bool solved = gpio_get(BUTTON_PIN) == 0;

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  sleep_ms(1000);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  sleep_ms(1000);
  printf("size of log_buffer_header_t %d\n", sizeof(log_buffer_header_t));
  printf("Board initialized!\n");

// START TESTING THE MAZE SOLVING CODE START
//   int v_walls[4][3] = {
//     {0, 0, -1},
//     {1, 0, -1},
//     {1, 0, 1},
//     {-1, -1, -1}
// };
// int h_walls[3][4] = {
//     {1, 1, 1, -1},
//     {0, -1, -1, -1},
//     {1, 1, 1, -1}
// };
//   buildGraph(h_walls, v_walls);
//   int* path = getShortestDistancePath(0, 9);
//   char* instructions = getPathInstructions(path);
//   return -1;
  // END TESTING THE MAZE SOLVING CODE END
  int res = init_log_flash();
  if (res != 0)
  {
    printf("error initing log flash\n");
  }

  // print_last(2);

  multicore_launch_core1(core1_entry);
  // Init motor output right and left (both forward)
  initMotors();
  encoders_register_callbacks();

  // print_last(100);

  if(!solved)
  {
    setYellowStatus();
    // Exploration phase
    explorationRun();
  } else {
    setGreenStatus();
    // We also need a run to go back to the start

    // Build our graph and get the best path instructions
    // we'll need some way to restart all this, since we're basically doing a new route back to start position
    // prob best to turn around first
    // state_t dest_state = mouseGetState();
    // int target = dest_state.x * MAZE_HEIGHT + dest_state.y;

    int8_t v[MAZE_HEIGHT][MAZE_WIDTH - 1];
    int8_t h[MAZE_HEIGHT - 1][MAZE_WIDTH];
    int target = get_target();
    int ret = read_walls(h,v);
    if (ret <0 || target == -1)
    {
      setRedStatus();
      return 0;
    }


    setVWalls(v);
    setHWalls(h);
    printMaze();
    printf("Target is %d\n", target);
    buildGraph(h, v);
    char* instructions = getPathInstructions(getShortestDistancePath(0, target), target);
    // // write this to robs buffer (instructions) (could also just write it while generating instructions)
    lfprintf("Shortest path instructions for mouse\n");
    lfprintf(instructions);
    solvedRun(instructions);
    smart_stop();
  }
  return 0;
}