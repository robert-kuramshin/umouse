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

#include "VL53L1X_api.h"
#include "VL53L1X_types.h"
#include <math.h>

#define RIGHT_MOTOR_A_PIN (15)
#define RIGHT_MOTOR_B_PIN (14)
#define RIGHT_ENCODER_A_PIN (12)
#define RIGHT_ENCODER_B_PIN (13)

#define LEFT_MOTOR_A_PIN (0)
#define LEFT_MOTOR_B_PIN (1)
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
float adcData(int adc_index) {
  adc_select_input(adc_index);
  float voltage = adc_read() * (3.3f / (1 << 12));
  return voltage;
}

void tuneV2D() {
  // At each distance chosen from i = 0 -> 5 inches?, with delta 0.5 inches
  // get ~100 voltage readings for i;
  // get v_variance, v_mean
  // build distance map as if voltage reading is between v_i_mean +/v_i_variance -> return i;
  // idk if this is the best way to deal with voltage noisiness, but based on readings seems pretty stable?

  printf("you have 10 seconds\n");
  sleep_ms(10000);
  float delta = 0.5;
  int max_dist = 5;
  int num_readings;
  int max_readings = 200;
  for (int i = 0; i < max_dist / delta; i++){
    printf("You have 3 seconds to set surface @ distance %f\n", i * delta);
    sleep_ms(5000);
    num_readings = 0;
    float total_voltage = 0;
    float total_voltage_squared = 0;
    while (num_readings < max_readings) {
      float data = adcData(RIGHT_ADC_INDEX);
      total_voltage += data;
      total_voltage_squared += (data * data);
      num_readings += 1;
      sleep_ms(500);
    }
    float mean = total_voltage / 50;
    float var = (total_voltage_squared - (total_voltage * total_voltage / max_readings)) / (max_readings - 1); 
    printf("Distance: %f has range low = %f and high = %f\n", i * delta, mean - var , mean + var);
  }
}

float rightVoltage2Distance(float data) {
// interpolate later, for now get crude dists
  float m;
  if (0.095 < data && data < 0.2) {
    m = data - 0.095;
    return 0.5 - m * 4.76;
  } else if (0.062 < data && data < 0.095) {
    m = data - 0.062;
    return 1.0 - m * 15.15;
  } else if (0.045 < data && data < 0.062) {
    m = data - 0.045;
    return 1.5 - m * 29.41;
  }
  return 2.0;
}
float leftVoltage2Distance(float data) {
  float m;
  if (0.05 < data && data < 0.07) {
    m = data - 0.05;
    return 0.5 - m * 25;
  } else if (0.032 < data && data < 0.05) {
    m = data - 0.032;
    return 1.0 - m * 27.77;
  } else if (0.0265 < data && data < 0.032) {
    m = data - 0.0265;
    return 1.5 - m * 90.9;
  }
  return 2.0;
}

void center_logic(uint16_t front_dist, float right_dist, float left_dist) {
  // take distance measurements and send motor commands to keep center
  // lets say right and left distance max is 1.5 inches
  int duty = 60;
  float tolerance = 0.2;
  float delta = right_dist - left_dist;
  printf("Delta:%f\n", delta);
  int sign = delta < 0 ? -1 : 1;
  delta = abs(delta);
  if (delta < tolerance) {
    // do nothing, we're centered
    goForward(duty);
    ;
  }
  // if delta > tol and sign == -1 -> increase duty in right wheel
  else if (sign == -1) {
    runRightMotor(duty + 10, 250);
  } else if (sign == 1) {
    runLeftMotor(duty + 10, 250);
  }
}

void runRightMotor(int duty, float duration) {
  printf("run motor");
  uint rslice;
  uint rchan;
  rslice = pwm_gpio_to_slice_num(RIGHT_MOTOR_A_PIN);
  rchan = pwm_gpio_to_channel(RIGHT_MOTOR_A_PIN);
  pwm_set_enabled(rslice, true);
  pwm_set_wrap(rslice, WRAP);
  pwm_set_duty(rslice, rchan, duty);
  sleep_ms(duration);
  // pwm_set_duty(rslice, rchan, 0);
}

void runLeftMotor(int duty, float duration) {
  uint lslice;
  uint lchan;
  lslice = pwm_gpio_to_slice_num(LEFT_MOTOR_A_PIN);
  lchan = pwm_gpio_to_channel(LEFT_MOTOR_A_PIN);
  pwm_set_enabled(lslice, true);
  pwm_set_wrap(lslice, WRAP);
  pwm_set_duty(lslice, lchan, duty);
  sleep_ms(duration);
  // pwm_set_duty(lslice, lchan, 0);
}

// 0 <= duty <= 100
void pwm_set_duty(uint slice_num, uint chan, int duty)
{
  pwm_set_chan_level(slice_num, chan, duty * WRAP / 100);
}

void goForward(int duty)
{
  uint rslice;
  uint rchan;
  uint lslice;
  uint lchan;
  rslice = pwm_gpio_to_slice_num(RIGHT_MOTOR_A_PIN);
  rchan = pwm_gpio_to_channel(RIGHT_MOTOR_A_PIN);
  pwm_set_enabled(rslice, true);
  pwm_set_wrap(rslice, WRAP);
  lslice = pwm_gpio_to_slice_num(LEFT_MOTOR_A_PIN);
  lchan = pwm_gpio_to_channel(LEFT_MOTOR_A_PIN);
  pwm_set_enabled(lslice, true);
  pwm_set_wrap(lslice, WRAP);

  pwm_set_duty(rslice, rchan, duty);
  pwm_set_duty(lslice, lchan, duty);
}

void goBackward(int duty)
{
  uint rslice;
  uint rchan;
  uint lslice;
  uint lchan;
  rslice = pwm_gpio_to_slice_num(RIGHT_MOTOR_B_PIN);
  rchan = pwm_gpio_to_channel(RIGHT_MOTOR_B_PIN);
  pwm_set_enabled(rslice, true);
  pwm_set_wrap(rslice, WRAP);
  lslice = pwm_gpio_to_slice_num(LEFT_MOTOR_B_PIN);
  lchan = pwm_gpio_to_channel(LEFT_MOTOR_B_PIN);
  pwm_set_enabled(lslice, true);
  pwm_set_wrap(lslice, WRAP);

  pwm_set_duty(rslice, rchan, duty);
  pwm_set_duty(lslice, lchan, duty);
}

void turn_left(int duty)
{
  uint rslice;
  uint rchan;
  uint lslice;
  uint lchan;
  rslice = pwm_gpio_to_slice_num(RIGHT_MOTOR_A_PIN);
  rchan = pwm_gpio_to_channel(RIGHT_MOTOR_A_PIN);
  pwm_set_enabled(rslice, true);
  pwm_set_wrap(rslice, WRAP);
  lslice = pwm_gpio_to_slice_num(LEFT_MOTOR_B_PIN);
  lchan = pwm_gpio_to_channel(LEFT_MOTOR_B_PIN);
  pwm_set_enabled(lslice, true);
  pwm_set_wrap(lslice, WRAP);

  pwm_set_duty(rslice, rchan, duty);
  pwm_set_duty(lslice, lchan, duty);
}

void turn_right(int duty)
{
  uint rslice;
  uint rchan;
  uint lslice;
  uint lchan;
  rslice = pwm_gpio_to_slice_num(RIGHT_MOTOR_B_PIN);
  rchan = pwm_gpio_to_channel(RIGHT_MOTOR_B_PIN);
  pwm_set_enabled(rslice, true);
  pwm_set_wrap(rslice, WRAP);
  lslice = pwm_gpio_to_slice_num(LEFT_MOTOR_A_PIN);
  lchan = pwm_gpio_to_channel(LEFT_MOTOR_A_PIN);
  pwm_set_enabled(lslice, true);
  pwm_set_wrap(lslice, WRAP);

  pwm_set_duty(rslice, rchan, duty);
  pwm_set_duty(lslice, lchan, duty);
}

void turn_test(bool turn_right)
{
  uint rslice;
  uint rchan;
  uint lslice;
  uint lchan;
  gpio_set_function(RIGHT_MOTOR_A_PIN, GPIO_FUNC_PWM);
  rslice = pwm_gpio_to_slice_num(RIGHT_MOTOR_A_PIN);
  rchan = pwm_gpio_to_channel(RIGHT_MOTOR_A_PIN);
  pwm_set_enabled(rslice, true);
  pwm_set_wrap(rslice, WRAP);

  gpio_set_function(LEFT_MOTOR_B_PIN, GPIO_FUNC_PWM);
  lslice = pwm_gpio_to_slice_num(LEFT_MOTOR_B_PIN);
  lchan = pwm_gpio_to_channel(LEFT_MOTOR_B_PIN);
  pwm_set_enabled(lslice, true);
  pwm_set_wrap(lslice, WRAP);

  pwm_set_duty(rslice, rchan, 60);
  pwm_set_duty(lslice, lchan, 60);
}

void basic_sequence_test()
{
  sleep_us(5 * 1000 * 1000);

  goForward(100);
  sleep_us(5 * 1000 * 1000);

  goForward(0);
  sleep_us(3 * 1000 * 1000);

  goBackward(100);
  sleep_us(5 * 1000 * 1000);

  goBackward(0);
  sleep_us(3 * 1000 * 1000);

  turn_left(100);
  sleep_us(7 * 1000 * 1000);

  turn_left(0);
  sleep_us(3 * 1000 * 1000);

  turn_right(100);
  sleep_us(7 * 1000 * 1000);

  turn_right(0);
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

int main() {
  stdio_init_all();
  adc_init();

  if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(1000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(1000);

  // Initialize adc pins
  adc_gpio_init(RIGHT_ADC_PIN);
  adc_gpio_init(LEFT_ADC_PIN);

  // tuneV2D();
  // return 1;
  // init tof sensor
  VL53L1X_Result_t results;
  VL53L1X_Status_t status = tof_init();
  printf("Done configuring sensor!\n");

  // // Init motor output right and left (both forward)
  gpio_set_function(RIGHT_MOTOR_A_PIN, GPIO_FUNC_PWM);
  gpio_set_function(RIGHT_MOTOR_B_PIN, GPIO_FUNC_PWM);
  gpio_set_function(LEFT_MOTOR_A_PIN, GPIO_FUNC_PWM);
  gpio_set_function(LEFT_MOTOR_B_PIN, GPIO_FUNC_PWM);

  gpio_pull_up(RIGHT_ENCODER_A_PIN);
  gpio_pull_up(RIGHT_ENCODER_B_PIN);
  gpio_pull_up(LEFT_ENCODER_A_PIN);
  gpio_pull_up(LEFT_ENCODER_B_PIN);
  // gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_b);
  // // basic_sequence_test();
  // printf("Freq %d\n", 125000000/WRAP);

  //Init interrupts
  // gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_LEVEL_HIGH, true, &gpio_callback_a);
  

  // Repeating timer
  repeating_timer_t timer;
  add_repeating_timer_us(-1 * MEASURE_PERIOD_SEC * SEC_TO_uS, timer_callback,NULL, &timer);
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
  bool first_range = true;
  int min_distance = 50;
  int duty;
  int count = 1;
  while (true) {
    // Wait until we have new data (front distance)
    uint8_t dataReady;
    do {
      status = VL53L1X_CheckForDataReady(I2C_DEV_ADDR, &dataReady);
      sleep_us(1);
    } while (dataReady == 0);

    // Get right distance
    float right_data = rightVoltage2Distance(adcData(RIGHT_ADC_INDEX));

    // Get left distance
    float left_data = leftVoltage2Distance(adcData(LEFT_ADC_INDEX));

    // Read and display result
    status += VL53L1X_GetResult(I2C_DEV_ADDR, &results);
    printf("Status = %2d, dist = %5d, rightV = %f, leftV = %f\n", results.status, results.distance, right_data, left_data);
    center_logic(results.distance, right_data, left_data);
    
    // int scale = 1000;
    // int error = (results.distance - min_distance) / scale;
    // if (error > 1) {
    //   error = 1;
    // } else if (error < 0) {
    //   error = 0;
    // }
    // float duty_scale = 60;
    // duty = duty_scale * error;
    // if (error > 0) {
    //   goForward(duty);
    // } else {
    //   turn_right(60);
    // }
    // 80mm ~ pi inches
    // Clear the sensor for a new measurement
    status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
    if (first_range) {  // Clear twice on first measurement
      status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
      first_range = false;
    }
    sleep_ms(500);
  }
}
