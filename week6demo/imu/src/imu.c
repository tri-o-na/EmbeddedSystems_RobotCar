#include "imu.h"
#include "hardware/i2c.h"
#include <stdio.h>

#define I2C_PORT i2c0
#define SDA_PIN 16
#define SCL_PIN 17
#define MPU_ADDR 0x68

#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B

void imu_init(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Wake device
    uint8_t buf[2] = { REG_PWR_MGMT_1, 0x00 };
    i2c_write_blocking(I2C_PORT, MPU_ADDR, buf, 2, false);
    printf("IMU init done\n");
}

static bool read_regs(uint8_t reg, uint8_t *dst, size_t n) {
    if (i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg, 1, true) < 0) return false;
    if (i2c_read_blocking(I2C_PORT, MPU_ADDR, dst, n, false) < 0) return false;
    return true;
}

bool imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t d[6];
    if (!read_regs(REG_ACCEL_XOUT_H, d, 6)) return false;
    *ax = (int16_t)((d[0] << 8) | d[1]);
    *ay = (int16_t)((d[2] << 8) | d[3]);
    *az = (int16_t)((d[4] << 8) | d[5]);
    return true;
}
