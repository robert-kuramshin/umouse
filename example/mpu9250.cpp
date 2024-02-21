
#include <stdio.h>
#include <stdlib.h>

#include "hardware/i2c.h"

#include "pico/bootrom.h"

#include "pico/stdlib.h"

#define MPU9250_I2C_ADDR (0x68)

#include "Arduino.h"
#include "Wire.h"
#include "MPU9250.h"

MPU9250 mpu; // You can also use MPU9255 as is

// static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
//     // For this particular device, we send the device the register we want to read
//     // first, then subsequently read from the device. The register is auto incrementing
//     // so we don't need to keep sending the register we want, just the first.

//     reg |= 1;
//     i2c_write_blocking(i2c0, MPU9250_I2C_ADDR, &reg, 1, false);
//     sleep_ms(10);
//     i2c_read_blocking(i2c0, MPU9250_I2C_ADDR, buf, len, false);
//     sleep_ms(10);
// }

// static void mpu9250_reset() {
//     // Two byte reset. First byte register, second byte data
//     // There are a load more options to set up the device in different ways that could be added here
//     uint8_t buf[] = {0x6B, 0x00};
//     i2c_write_blocking(i2c0, MPU9250_I2C_ADDR,  buf, 2, false);
// }


// static void mpu9250_read_raw(int16_t accel[3], int16_t gyro[3], int16_t mag[3], int16_t *temp) {
//     uint8_t buffer[6];

//     // Start reading acceleration registers from register 0x3B for 6 bytes
//     read_registers(0x3B, buffer, 6);

//     for (int i = 0; i < 3; i++) {
//         accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
//     }

//     // Now gyro data from reg 0x43 for 6 bytes
//     read_registers(0x43, buffer, 6);

//     for (int i = 0; i < 3; i++) {
//         gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
//     }

//     // Now mag data from reg 0x03 for 6 bytes
//     read_registers(0x03, buffer, 6);

//     for (int i = 0; i < 3; i++) {
//         mag[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
//     }

//     // Now temperature from reg 0x41 for 2 bytes
//     read_registers(0x41, buffer, 2);

//     *temp = buffer[0] << 8 | buffer[1];
// }


int main() {
    stdio_init_all();

    // i2c_init(i2c0, 100000);

    // gpio_set_function(16, GPIO_FUNC_I2C);
    // gpio_set_function(17, GPIO_FUNC_I2C);

    // gpio_pull_up(16);
    // gpio_pull_up(17);
    Wire.begin();

    sleep_ms(3000);


    printf("Wires %d %d\n",PIN_WIRE0_SDA,PIN_WIRE0_SCL);

    mpu.verbose(true);
    mpu.setup(0x68);

    mpu.calibrateAccelGyro();

    // mpu.setMagBias(-439.79,33.83,-161.86);
    // mpu.setMagScale(0.94,1.34,0.84);

    // mpu.setMagBias(-507.45, 55.20, 67.16);
    // mpu.setMagScale(0.91, 1.07, 1.03);


    // mpu.setMagBias(-489.64, 14.24, 92.99);
    // mpu.setMagScale(0.88, 1.12, 1.03 );

    // mpu.setMagBias(-507.45,81.90,122.26);
    // mpu.setMagScale(0.92, 1.07, 1.02 );

    mpu.calibrateMag();

    for(int i =0;i<20000;i++){
        if (mpu.update()) {
            int16_t x = mpu.getMagX();
            int16_t y = mpu.getMagY();
            float m = atan2(x, -y) * 180 / PI + 180;
            Serial.print(mpu.getYaw()); Serial.print(", ");
            Serial.print(mpu.getPitch()); Serial.print(", ");
            Serial.println(mpu.getRoll());
        }
        sleep_ms(12);
    }

    // mpu9250_reset();

    // uint8_t id;
    // read_registers(0x75, &id, 1);
    // printf("I2C address is 0x%x\n", id);

    // read_registers(0x00, &id, 1);
    // printf("AKM whoami is 0x%x\n", id);

    // uint8_t data = 0x22;
    // i2c_write_blocking(i2c0, MPU9250_I2C_ADDR, INT_PIN_CFG, &data, 1, false);
    // data = 0x01;
    // i2c_write_blocking(i2c0, MPU9250_I2C_ADDR, INT_ENABLE, &data, 1, false);  // Enable data ready (bit 0) interrupt

    // int16_t acceleration[3] = {0};
    // int16_t gyro[3] = {0};
    // int16_t mag[3] = {0};
    // int16_t temp = 0;

    // for(int i = 0; i<5;i++)
    // {
    //     mpu9250_read_raw(acceleration, gyro, mag, &temp);

    //     // These are the raw numbers from the chip, so will need tweaking to be really useful.
    //     // See the datasheet for more information
    //     printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
    //     printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
    //     printf("Mag. X = %d, Y = %d, Z = %d\n", mag[0], mag[1], mag[2]);
    //     // Temperature is simple so use the datasheet calculation to get deg C.
    //     // Note this is chip temperature.
    //     printf("Temp. = %f\n", (temp / 340.0) + 36.53);
    //     sleep_ms(2000);
    // }

    


    reset_usb_boot(0, 0);
}