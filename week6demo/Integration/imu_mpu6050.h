#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float heading;
} imu_state_t;

bool imu_init(void);
bool imu_read(imu_state_t *state);

#endif
