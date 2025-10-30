#include "imu_mpu6050.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"      
#include <stdint.h>          
#include <math.h>


#define I2C_PORT i2c0
#define PIN_SDA 4
#define PIN_SCL 5
#define MPU_ADDR 0x68

static absolute_time_t last_read;

bool imu_init(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    uint8_t buf[2] = {0x6B, 0x00};
    if (i2c_write_blocking(I2C_PORT, MPU_ADDR, buf, 2, false) < 0)
        return false;

    last_read = get_absolute_time();
    return true;
}

bool imu_read(imu_state_t *s) {
    uint8_t reg = 0x3B;
    uint8_t data[14];
    if (i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg, 1, true) < 0) return false;
    if (i2c_read_blocking(I2C_PORT, MPU_ADDR, data, 14, false) < 0) return false;

    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];
    int16_t gz = (data[12] << 8) | data[13];

    s->ax = ax / 16384.0f;
    s->ay = ay / 16384.0f;
    s->az = az / 16384.0f;
    s->gz = gz / 131.0f;

    // integrate gyro-Z for heading
    absolute_time_t now = get_absolute_time();
    float dt = absolute_time_diff_us(last_read, now) / 1e6f;
    last_read = now;
    s->heading += s->gz * dt;
    if (s->heading > 180) s->heading -= 360;
    if (s->heading < -180) s->heading += 360;
    return true;
}
