#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>


typedef struct {
    float x;      // meters
    float y;      // meters
    float theta;  // radians, 0 = facing +X
} Pose2D;

void odom_init(Pose2D *p);
void odom_update(Pose2D *p,
                 int32_t dleft_ticks,
                 int32_t dright_ticks);

#endif
