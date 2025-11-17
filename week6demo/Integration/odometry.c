#include "odometry.h"
#include <math.h>

// ====== FILL THESE WITH YOUR REAL VALUES ======
#define TICKS_PER_REV   20.0f    // encoder pulses per wheel revolution
#define WHEEL_RADIUS_M  0.033f   // wheel radius, meters (e.g. 33 mm)
#define WHEEL_BASE_M    0.10f    // distance between wheels, meters
// ==============================================

static const float M_PER_TICK =
    (2.0f * (float)M_PI * WHEEL_RADIUS_M) / TICKS_PER_REV;

void odom_init(Pose2D *p)
{
    p->x = 0.0f;
    p->y = 0.0f;
    p->theta = 0.0f;
}

static float normalize_angle(float a)
{
    while (a > (float)M_PI)   a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI)  a += 2.0f * (float)M_PI;
    return a;
}

void odom_update(Pose2D *p,
                 int32_t dleft_ticks,
                 int32_t dright_ticks)
{
    float dl = dleft_ticks  * M_PER_TICK;
    float dr = dright_ticks * M_PER_TICK;

    float dc     = 0.5f * (dl + dr);             // forward distance
    float dtheta = (dr - dl) / WHEEL_BASE_M;     // change in heading

    p->theta = normalize_angle(p->theta + dtheta);

    p->x += dc * cosf(p->theta);
    p->y += dc * sinf(p->theta);
}
