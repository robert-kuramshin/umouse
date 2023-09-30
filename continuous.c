/*
* Example usage of the VL53L1X Pico Library for Pico/Pico W
*
* This program will continously print out packets read from the VL53L1X sensor
* to USB (to read them, leave the Pico/Pico W connected to your computer via USB
* and then open a serial connection to the corresponding port with baudrate 115200).
*/

/*
* Copyright 2022, Alex Mous
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "haw/MPU6050.h"

#ifdef PICO_W_BOARD
#include "pico/cyw43_arch.h"
#endif

#include "VL53L1X_api.h"
#include "VL53L1X_types.h"


#define I2C_DEV_ADDR 0x29

// By default these devices  are on bus address 0x6b
static int addr = 0x68;
static int a_addr = 0x1c;
// #ifdef i2c_default
static void mpu6050_reset() {
    // write 0 to sleep register to come off low power mode. datasheet pg 41/48
    uint8_t buf[] = {0x6B, 0x0}; // 0x80 previous reset instruction
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
    // set G scale
    // i2c_write_blocking(i2c_default, 0x1c, &a, 1, true);
    // i2c_read_blocking(i2c->instance, i2c->address, buf, len, false);
    // set A range
    uint8_t a_buff;
    i2c_write_blocking(i2c_default, addr, &a_addr, 1, true);
    i2c_read_blocking(i2c_default, addr, a_buff, 1, false);
    a_buff &= 0xE7;
    a_buff |= (0 << 3);
    uint8_t data[2] = {0x1C, a_buff};
    i2c_write_blocking(i2c_default, addr, data, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}
// #endif

// MPU code
int main() {
    stdio_init_all();

    // tof sensor
//     VL53L1X_Status_t status;
//     VL53L1X_Result_t results;
//   /////////////////////////////////////////////
//   printf("about to init.\n");
//   // GPIO sda=2, scl=3, i2c1
//   if (VL53L1X_I2C_Init(I2C_DEV_ADDR, i2c1) < 0) {
//       printf("Error initializing sensor.\n");
//       return 0;
//   }
//   printf("done init.\n");
//   // Ensure the sensor has booted
//   uint8_t sensorState;
//   do {
//     status += VL53L1X_BootState(I2C_DEV_ADDR, &sensorState);
//     VL53L1X_WaitMs(I2C_DEV_ADDR, 2);
//     printf("cmon\n");
//   } while (sensorState == 0);
//   printf("Sensor booted.\n");

//   // Initialize and configure sensor
//   status = VL53L1X_SensorInit(I2C_DEV_ADDR);
//   status += VL53L1X_SetDistanceMode(I2C_DEV_ADDR, 1);
//   status += VL53L1X_SetTimingBudgetInMs(I2C_DEV_ADDR, 100);
//   status += VL53L1X_SetInterMeasurementInMs(I2C_DEV_ADDR, 100);
//   status += VL53L1X_StartRanging(I2C_DEV_ADDR);

//   // Measure and print continuously
//   bool first_range = true;
//   while (1) {
//     // Wait until we have new data
//     uint8_t dataReady;
//     do {
//       status = VL53L1X_CheckForDataReady(I2C_DEV_ADDR, &dataReady);
//       sleep_us(1);
//     } while (dataReady == 0);

//     // Read and display result
//     status += VL53L1X_GetResult(I2C_DEV_ADDR, &results);
//     printf("Status = %2d, dist = %5d, Ambient = %2d, Signal = %5d, #ofSpads = %5d\n",
//       results.status, results.distance, results.ambient, results.sigPerSPAD, results.numSPADs);
//     // 80mm ~ pi inches
//     // Clear the sensor for a new measurement
//     status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
//     if (first_range) {  // Clear twice on first measurement
//       status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
//       first_range = false;
//     }
//     sleep_ms(200);
//   }

    
    const int sda = 4;
    const int scl = 5;
    // I2C0 bus on the gpio4 & 5
    i2c_init(i2c_default, 400 * 1000);
    gpio_init(sda);
    gpio_init(scl);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda); 
    gpio_pull_up(scl);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(sda, scl, GPIO_FUNC_I2C));
    mpu6050_t mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_GND);

    if (mpu6050_begin(&mpu6050))
    {
        // Set scale of gyroscope
        mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
        // Set range of accelerometer
        mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);

        // Enable gyroscope and accelerometer readings
        mpu6050_set_gyroscope_measuring(&mpu6050, true);
        mpu6050_set_accelerometer_measuring(&mpu6050, true);

        // Enable free fall, motion and zero motion interrupt flags
        mpu6050_set_int_free_fall(&mpu6050, false);
        mpu6050_set_int_motion(&mpu6050, false);
        mpu6050_set_int_zero_motion(&mpu6050, false);

        // Set motion detection threshold and duration
        mpu6050_set_motion_detection_threshold(&mpu6050, 2);
        mpu6050_set_motion_detection_duration(&mpu6050, 5);

        // Set zero motion detection threshold and duration
        mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);
        mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);
    }
    else
    {
        while (1)
        {
            // Endless loop
            printf("Error! MPU6050 could not be initialized. Make sure you've entered the correct address. And double check your connections.");
            return -1;
            sleep_ms(500);
        }
    }

    while (1)
    {
        // Fetch all data from the sensor | I2C is only used here
        mpu6050_event(&mpu6050);

        // Pointers to float vectors with all the results
        mpu6050_vectorf_t *accel = mpu6050_get_accelerometer(&mpu6050);
        mpu6050_vectorf_t *gyro = mpu6050_get_gyroscope(&mpu6050);

        // Activity struct holding all interrupt flags
        mpu6050_activity_t *activities = mpu6050_read_activities(&mpu6050);

        // Print all the measurements
        printf("Ax = %f, Ay = %f, Az = %f\n", accel->x, accel->y, accel->z);
        printf("Gx = %f, Gy = %f, Gz = %f\n\n", gyro->x, gyro->y, gyro->z);
        // // Print all motion interrupt flags
        // printf("Overflow: %d - Freefall: %d - Inactivity: %d, Activity: %d, DataReady: %d\n",
        //        activities->isOverflow,
        //        activities->isFreefall,
        //        activities->isInactivity,
        //        activities->isActivity,
        //        activities->isDataReady);

        // // Print all motion detect interrupt flags
        // printf("PosX: %d - NegX: %d -- PosY: %d - NegY: %d -- PosZ: %d - NegZ: %d\n",
        //        activities->isPosActivityOnX,
        //        activities->isNegActivityOnX,
        //        activities->isPosActivityOnY,
        //        activities->isNegActivityOnY,
        //        activities->isPosActivityOnZ,
        //        activities->isNegActivityOnZ);

        sleep_ms(1000);
    }

    // mpu6050_reset();
    // int16_t acceleration[3], gyro[3], temp;

    // while (1) {
    //     mpu6050_read_raw(acceleration, gyro, &temp);
    //     printf("Ax = %d, Ay = %d, Az = %d\n", acceleration[0], acceleration[1], acceleration[2]);
    //     printf("Gx = %d, Gy = %d, Gz = %d\n", gyro[0], gyro[1], gyro[2]);
    //     sleep_ms(500);
    // }
}

// void startup() {
//   stdio_init_all();
//   for (int i = 0; i < 20; i ++){
//     printf("drip too cold\n");
//     sleep_ms(1000);
//   }

//   // Blink the onboard LED to show booting
// #if PICO_W_BOARD
//   if (cyw43_arch_init()) {
//     printf("Failed to initialize Pico W.\n");
//     return 1;
//   }
// #else
//   gpio_init(PICO_DEFAULT_LED_PIN);
//   gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
// #endif

//   for (int i=0; i<10; i++) {
// #ifdef PICO_W_BOARD
//     cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
// #else
//     gpio_put(PICO_DEFAULT_LED_PIN, 1);
// #endif

//     sleep_ms(250);

// #ifdef PICO_W_BOARD
//     cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
// #else
//     gpio_put(PICO_DEFAULT_LED_PIN, 0);
// #endif

//     sleep_ms(250);
//   }
// }