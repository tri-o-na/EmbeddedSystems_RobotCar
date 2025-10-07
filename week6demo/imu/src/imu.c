#include "imu.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define I2C_PORT   i2c0
#define SDA_PIN    16
#define SCL_PIN    17
#define ACC_ADDR   0x19

// Registers (LIS3DH / LSM303 accel family)
#define REG_STATUS_A   0x27
#define REG_CTRL1_A    0x20
#define REG_CTRL2_A    0x21
#define REG_CTRL4_A    0x23
#define REG_OUT_X_L_A  0x28

// Simple EMA smoothing factor (0..1). Smaller = smoother.
#define ALPHA 0.15f

static float ax_f=0, ay_f=0, az_f=0;

static inline bool i2c_w(uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    return i2c_write_blocking(I2C_PORT, ACC_ADDR, b, 2, false) >= 0;
}
static inline bool i2c_r(uint8_t reg, uint8_t *dst, size_t n) {
    uint8_t ra = reg | 0x80; // auto-increment
    if (i2c_write_blocking(I2C_PORT, ACC_ADDR, &ra, 1, true) < 0) return false;
    return i2c_read_blocking(I2C_PORT, ACC_ADDR, dst, n, false) >= 0;
}

void imu_init(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(5);

    // CTRL1_A: ODR=10Hz (0010), LPen=0, XYZ enable = 111  => 0b0010 0111 = 0x27
    // If you want 25Hz, use 0x37; 50Hz, 0x47.
    i2c_w(REG_CTRL1_A, 0x27);

    // CTRL2_A: high-pass disabled / normal mode (0x00 is fine)
    i2c_w(REG_CTRL2_A, 0x00);

    // CTRL4_A: BDU=1 (bit7), HR=1 (bit3), ±2g (FS=00)
    // 0b1000 1000 = 0x88
    i2c_w(REG_CTRL4_A, 0x88);

    sleep_ms(10);
    printf("IMU init done (10Hz, BDU, HR, +/-2g)\n");
}

static bool read_raw_xyz(int16_t *ax, int16_t *ay, int16_t *az) {
    // Wait for fresh sample so we don't read mid-update
    uint8_t st = 0;
    if (!i2c_r(REG_STATUS_A, &st, 1)) return false;
    if ((st & 0x08) == 0) return false; // ZYXDA = 1 => new X/Y/Z available

    uint8_t d[6];
    if (!i2c_r(REG_OUT_X_L_A, d, 6)) return false;

    // Little-endian: L then H
    *ax = (int16_t)(d[1] << 8 | d[0]);
    *ay = (int16_t)(d[3] << 8 | d[2]);
    *az = (int16_t)(d[5] << 8 | d[4]);
    return true;
}

bool imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az) {
    int16_t x,y,z;
    if (!read_raw_xyz(&x,&y,&z)) return false;

    // Optional: EMA smoothing (still return raw ints to your caller if you prefer)
    // Convert to float g first (±2g => 16384 counts per g)
    float xf = x / 16384.0f;
    float yf = y / 16384.0f;
    float zf = z / 16384.0f;

    ax_f = (1.0f - ALPHA) * ax_f + ALPHA * xf;
    ay_f = (1.0f - ALPHA) * ay_f + ALPHA * yf;
    az_f = (1.0f - ALPHA) * az_f + ALPHA * zf;

    // If you must return integers, re-scale; otherwise expose floats in a new API.
    *ax = (int16_t)(ax_f * 16384.0f);
    *ay = (int16_t)(ay_f * 16384.0f);
    *az = (int16_t)(az_f * 16384.0f);
    return true;
}
