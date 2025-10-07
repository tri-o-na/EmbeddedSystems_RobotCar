#ifndef IMU_H
#define IMU_H

#include "pico/stdlib.h"
#include <stdint.h>

// Init MPU6050 over I2C0 (SDA=GPIO4, SCL=GPIO5 by default)
void imu_init(void);

// Read raw accelerometer values (X,Y,Z)
bool imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az);

#endif
