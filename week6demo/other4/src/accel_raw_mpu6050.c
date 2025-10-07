#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

#define I2C_PORT i2c0
#define SDA_PIN  16   // <-- your SDA pin
#define SCL_PIN  17   // <-- your SCL pin
#define MPU6050_ADDR 0x68

// Initialize MPU6050
static void mpu6050_init(void) {
    uint8_t wake_cmd[] = {0x6B, 0x00};  // PWR_MGMT_1 register, wake up device
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, wake_cmd, 2, false);
}

// Read 6 raw accel bytes
static void read_raw_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = 0x3B;
    uint8_t buf[6];

    // Point to ACCEL_XOUT_H
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, 6, false);

    *ax = (buf[0] << 8) | buf[1];
    *ay = (buf[2] << 8) | buf[3];
    *az = (buf[4] << 8) | buf[5];
}

int main() {
    stdio_init_all();
    i2c_init(I2C_PORT, 400 * 1000);  // 400kHz I²C bus speed

    // Assign I2C functions to your chosen pins
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    mpu6050_init();
    printf("Raw accelerometer demo started (Pins 16/17)\n");

    int16_t ax, ay, az;

    while (true) {
        read_raw_accel(&ax, &ay, &az);
        printf("Raw accel X:%6d  Y:%6d  Z:%6d\n", ax, ay, az);
        sleep_ms(500);
    }
}
