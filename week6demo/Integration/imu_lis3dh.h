#ifndef IMU_LIS3DH_H
#define IMU_LIS3DH_H

#include <stdbool.h>

typedef struct {
    float ax, ay, az;
    float heading;
} imu_state_t;

bool imu_init(void);
bool imu_read(imu_state_t *s);

#endif
