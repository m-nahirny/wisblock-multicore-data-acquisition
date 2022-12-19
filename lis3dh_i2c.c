/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
//#include "pico/multicore.h"
#include "D:\\Pico\\pico-sdk\\src\\rp2_common\\pico_multicore\\include\\pico\\multicore.h"

/* Example code to talk to a LIS3DH Mini GPS module.

   This example reads data from all 3 axes of the accelerometer and uses an auxillary ADC to output temperature values.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is 4 (physical pin 6)) -> SDA on LIS3DH board
   GPIO PICO_DEFAULT_I2C_SCK_PIN (On Pico this is 5 (physical pin 7)) -> SCL on LIS3DH board
   3.3v (physical pin 36) -> VIN on LIS3DH board
   GND (physical pin 38)  -> GND on LIS3DH board
*/

#define LED_GREEN 23

// By default this device is on bus address 0x18

const int ADDRESS = 0x18;
const uint8_t TEMP_CFG_REG = 0x1F;
const uint8_t CTRL_REG_1 = 0x20;
const uint8_t CTRL_REG_2 = 0x21;
const uint8_t CTRL_REG_3 = 0x22;
const uint8_t CTRL_REG_4 = 0x23;
const uint8_t CTRL_REG_6 = 0x25;
const uint8_t WHO_AM_I = 0x0F;
const uint8_t OUT_T_L = 0x0C; // 16 bit resolution
const uint8_t OUT_T_H = 0x0D; // 16 bit resolution
const uint8_t OUT_T = 0x26; // 8 bit resolution
const uint8_t OUT_X_L = 0x28; 
const uint8_t OUT_X_H = 0x29; 
const uint8_t OUT_Y_L = 0x2A; 
const uint8_t OUT_Y_H = 0x2B; 
const uint8_t OUT_Z_L = 0x2C; 
const uint8_t OUT_Z_H = 0x2D; 
//const uint8_t TEMP_CFG_REG = 0xC0;

#define i2c_rak i2c1

const uint I2C0SDA = 20;
const uint I2C0SCL = 21;
const uint I2C1SDA = 2;
const uint I2C1SCL = 3;

void lis3dh_init() {
    uint8_t buf[2];

    buf[0] = TEMP_CFG_REG;
    // enable temperature sensor
    buf[1] = 0xC0;
    i2c_write_blocking(i2c_rak, ADDRESS, buf, 2, false);

    buf[0] = CTRL_REG_1;
    // 200 Hz high performance mode
    buf[1] = 0x67;
    i2c_write_blocking(i2c_rak, ADDRESS, buf, 2, false);

    buf[0] = CTRL_REG_4;
    // +/- 2g, high resolution
    // Turn block data update on (for temperature sensing)
    buf[1] = 0x88;
    i2c_write_blocking(i2c_rak, ADDRESS, buf, 2, false);

    // Configure ADC on RP2040 to read its temperature sensor
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}

void lis3dh_calc_value(uint16_t raw_value, float *final_value, bool isAccel) {
    // Convert with respect to the value being temperature or acceleration reading 
    float scaling;
    float senstivity = 0.004f; // g per unit

    if (isAccel == true) {
        scaling = 64 / senstivity;
    } else {
        scaling = 32;
//        scaling = 64;
    }

    // raw_value is signed
    *final_value = (float) ((int16_t) raw_value) / scaling;
}

void lis3dh_read_data(uint8_t reg, float *final_value, bool IsAccel) {
    // Read two bytes of data and store in a 16 bit data structure
    uint8_t lsb;
    uint8_t msb;
    uint16_t raw_accel;
    i2c_write_blocking(i2c_rak, ADDRESS, &reg, 1, true);
    int numRead = i2c_read_blocking(i2c_rak, ADDRESS, &lsb, 1, false);
    if (numRead == PICO_ERROR_GENERIC)
        puts("No device present");

    reg |= 0x01;
    i2c_write_blocking(i2c_rak, ADDRESS, &reg, 1, true);
    numRead = i2c_read_blocking(i2c_rak, ADDRESS, &msb, 1, false);
    if (numRead == PICO_ERROR_GENERIC)
        puts("No device present");

    raw_accel = (msb << 8) | lsb;
//    printf("Raw data %x\n", raw_accel);

    lis3dh_calc_value(raw_accel, final_value, IsAccel);
}

uint8_t who_am_I()
{
    uint8_t val;
    uint8_t reg = WHO_AM_I;

    int numWritten = i2c_write_blocking(i2c_rak, ADDRESS, &reg, 1, true);
    if (numWritten == PICO_ERROR_GENERIC)
        puts("No device present");
    int numRead = i2c_read_blocking(i2c_rak, ADDRESS, &val, 1, false);
    if (numRead == PICO_ERROR_GENERIC)
        puts("No device present");
    return val;
}

void core1_accel() {
    puts("Initializing I2C");
    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    uint res = i2c_init(i2c_rak, 400 * 1000);
    printf("I2C rate is %d\n", res);

    gpio_set_function(I2C1SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1SDA);
    gpio_pull_up(I2C1SCL);

    float x_accel, y_accel, z_accel, temp;

    printf("Hello, LIS3DH %x! Reading raw data from registers...\n", who_am_I());

    sleep_ms(2000);

    lis3dh_init();

    while (1) {
        lis3dh_read_data(OUT_X_L, &x_accel, true);
        lis3dh_read_data(OUT_Y_L, &y_accel, true);
        lis3dh_read_data(OUT_Z_L, &z_accel, true);
        lis3dh_read_data(OUT_T_L, &temp, false);

        // Display data 
        printf("TEMPERATURE: %.3f C\n", temp);
        // Acceleration is read as a multiple of g (gravitational acceleration on the Earth's surface)
        printf("ACCELERATION VALUES: X: %.3fg, Y: %.3fg, Z: %.3fg\n", x_accel, y_accel, z_accel);

        sleep_ms(500);

        // Clear terminal 
        //printf("\e[1;1H\e[2J");
    }
}

int main() {
    stdio_init_all();

    multicore_launch_core1(core1_accel);

    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);

    while (1) {
        // read the internal temperature sensor
        uint16_t raw = adc_read();
        const float conversion_factor = 3.3f / (1<<12);
        float result = raw * conversion_factor;
        float temp = 27 - (result -0.706)/0.001721;
        printf("RP2040  raw = %x, Temp = %f C\n", raw, temp);
        sleep_ms(1000);
    }

    return 0;
}
