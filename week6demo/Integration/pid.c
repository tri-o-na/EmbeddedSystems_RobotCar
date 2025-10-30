#include "pid.h"

void pid_init(PID *p, float kp, float ki, float kd, float min_out, float max_out) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->setpoint = 0; p->prev_error = 0; p->integral = 0;
    p->min_out = min_out; p->max_out = max_out;
}

float pid_update(PID *p, float input) {
    float error = p->setpoint - input;
    p->integral += error;
    float derivative = error - p->prev_error;
    p->prev_error = error;
    float out = p->kp * error + p->ki * p->integral + p->kd * derivative;
    if (out > p->max_out) out = p->max_out;
    if (out < p->min_out) out = p->min_out;
    return out;
}
