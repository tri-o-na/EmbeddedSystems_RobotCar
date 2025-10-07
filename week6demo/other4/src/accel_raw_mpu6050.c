#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// --- I2C pins (adjust) ---
#define I2C_PORT i2c0
#define PIN_SDA  4
#define PIN_SCL  5
#define MPU_ADDR 0x68

static void mpu_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(I2C_PORT, MPU_ADDR, buf, 2, false);
}

static void mpu_read(uint8_t reg, uint8_t *dst, size_t n) {
    i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDR, dst, n, false);
}

static int16_t to_i16(uint8_t msb, uint8_t lsb) {
    int16_t v = (int16_t)((msb << 8) | lsb);
    return v;
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    // Init I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    // Wake MPU6050: PWR_MGMT_1 (0x6B) = 0
    mpu_write(0x6B, 0x00);
    sleep_ms(50);

    printf("MPU6050 raw accel demo (no processing)\n");
    while (true) {
        uint8_t b[6];
        mpu_read(0x3B, b, 6);  // ACCEL_X/Y/Z H:L
        int16_t ax = to_i16(b[0], b[1]);
        int16_t ay = to_i16(b[2], b[3]);
        int16_t az = to_i16(b[4], b[5]);
        // Raw counts; default scale ~16384 LSB/g at ±2g
        printf("AX=%d AY=%d AZ=%d\n", ax, ay, az);
        sleep_ms(200);
    }
}
