
#include <stdio.h>
#include <stdlib.h>

#include "hardware/i2c.h"

#include "pico/bootrom.h"

#include "pico/stdlib.h"

#include "pico/float.h"

// #define MPU9250_I2C_ADDR (0x68)

// #include "Arduino.h"
// #include "Wire.h"
// #include "MPU9250.h"

// MPU9250 mpu; // You can also use MPU9255 as is


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
#define HMC5883L_ADDR 0x1E
#define CONFIG_REG_A 0x00
#define CONFIG_REG_B 0x01
#define MODE_REG 0x02
#define DATA_REG 0x03


static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= 1;
    i2c_write_blocking(i2c0, HMC5883L_ADDR, &reg, 1, false);
    sleep_ms(10);
    i2c_read_blocking(i2c0, HMC5883L_ADDR, buf, len, false);
    sleep_ms(10);
}

#define LSB_TO_UT (100.0 / 1090.0)

int main() {
    stdio_init_all();

    i2c_init(i2c0, 100000);

    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_set_function(17, GPIO_FUNC_I2C);

    gpio_pull_up(16);
    gpio_pull_up(17);
    // Wire.begin();

    sleep_ms(3000);

    uint8_t data[2] = {CONFIG_REG_A, 0x70};
    i2c_write_blocking(i2c0, HMC5883L_ADDR, data, 2, false);
    data[0] = CONFIG_REG_B;
    data[1] = 0x20;
    i2c_write_blocking(i2c0, HMC5883L_ADDR, data, 2, false);
    data[0] = MODE_REG;
    data[1] = 0x00;
    i2c_write_blocking(i2c0, HMC5883L_ADDR, data, 2, false);

    float x,y,z;
    uint8_t rawdata[6] = {0};

    float min[3] = {0};
    float max[3] = {0};

    for(int i =0;i<1000;i++){
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

        // x -= -28.027523;
        // y -= -8.944954;
        // z -= 11.972478;

        float heading = atan2f(y, x);

        heading = heading * 180 / 3.14159265;

        if (heading < 0)
        {
            heading += 360;
        }

        if (x < min[0])
            min[0] = x;
        if (y < min[1])
            min[1] = y;
        if (z < min[2])
            min[2] = z;
        if (x > max[0])
            max[0] = x;
        if (y > max[1])
            max[1] = y;
        if (z > max[2])
            max[2] = z;
        printf("Magnetic field in X: %.2f uT, Y: %.2f uT, Z: %.2f uT, Heading: %.2f°\n", x, y, z, heading);
        sleep_ms(2);
    }
    float x_offset = (max[0] + min[0]) / 2;
    float y_offset = (max[1] + min[1]) / 2;
    float z_offset = (max[2] + min[2]) / 2;

    printf("%f %f %f\n", x_offset, y_offset, z_offset);

    // mpu.setMagBias(-439.79,33.83,-161.86);
    // mpu.setMagScale(0.94,1.34,0.84);

    // mpu.setMagBias(-507.45, 55.20, 67.16);
    // mpu.setMagScale(0.91, 1.07, 1.03);


    // mpu.setMagBias(-489.64, 14.24, 92.99);
    // mpu.setMagScale(0.88, 1.12, 1.03 );

    // mpu.setMagBias(-507.45,81.90,122.26);
    // mpu.setMagScale(0.92, 1.07, 1.02 );

    // mpu.calibrateMag();

    // for(int i =0;i<20000;i++){
    //     if (mpu.update()) {
    //         int16_t x = mpu.getMagX();
    //         int16_t y = mpu.getMagY();
    //         float m = atan2(x, -y) * 180 / PI + 180;
    //         Serial.print(mpu.getYaw()); Serial.print(", ");
    //         Serial.print(mpu.getPitch()); Serial.print(", ");
    //         Serial.println(mpu.getRoll());
    //     }
    //     sleep_ms(12);
    // }

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