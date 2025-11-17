#include "imu_lis3dh.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

#define I2C_PORT i2c0
#define PIN_SDA 16
#define PIN_SCL 17
#define ACC_ADDR 0x19

// Registers (LIS3DH / LSM303 accel family)
#define REG_STATUS_A   0x27
#define REG_CTRL1_A    0x20
#define REG_CTRL2_A    0x21
#define REG_CTRL4_A    0x23
#define REG_OUT_X_L_A  0x28

#define ALPHA 0.15f  // smoothing factor

static float ax_f=0, ay_f=0, az_f=0;

bool imu_init(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    sleep_ms(5);

    uint8_t ctrl1[2] = {REG_CTRL1_A, 0x27}; // normal mode, 10Hz, XYZ enable
    uint8_t ctrl2[2] = {REG_CTRL2_A, 0x00}; // no high-pass
    uint8_t ctrl4[2] = {REG_CTRL4_A, 0x88}; // BDU=1, HR=1, Â±2g
    i2c_write_blocking(I2C_PORT, ACC_ADDR, ctrl1, 2, false);
    i2c_write_blocking(I2C_PORT, ACC_ADDR, ctrl2, 2, false);
    i2c_write_blocking(I2C_PORT, ACC_ADDR, ctrl4, 2, false);

    printf("IMU (LIS3DH) initialized on SDA=%d SCL=%d addr=0x%02X\n", PIN_SDA, PIN_SCL, ACC_ADDR);
    return true;
}

bool imu_read(imu_state_t *s) {
    uint8_t ra = REG_OUT_X_L_A | 0x80;
    uint8_t d[6];
    if (i2c_write_blocking(I2C_PORT, ACC_ADDR, &ra, 1, true) < 0) return false;
    if (i2c_read_blocking(I2C_PORT, ACC_ADDR, d, 6, false) < 0) return false;

    int16_t x = (int16_t)(d[1] << 8 | d[0]);
    int16_t y = (int16_t)(d[3] << 8 | d[2]);
    int16_t z = (int16_t)(d[5] << 8 | d[4]);

    // Convert to g's
    float xf = x / 16384.0f;
    float yf = y / 16384.0f;
    float zf = z / 16384.0f;

    ax_f = (1.0f - ALPHA) * ax_f + ALPHA * xf;
    ay_f = (1.0f - ALPHA) * ay_f + ALPHA * yf;
    az_f = (1.0f - ALPHA) * az_f + ALPHA * zf;

    s->ax = ax_f;
    s->ay = ay_f;
    s->az = az_f;

    // Calculate pseudo-heading (tilt-based)
    s->heading = atan2f(s->ay, s->ax) * 180.0f / M_PI;

    return true;
}