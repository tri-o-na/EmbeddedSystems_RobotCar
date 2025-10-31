#include "pid.h"

void pid_init(PID *p, float kp, float ki, float kd, float out_min, float out_max) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->setpoint = 0.0f;
    p->integ = 0.0f; p->last = 0.0f;
    p->out_min = out_min; p->out_max = out_max;
}

float pid_update(PID *p, float measurement) {
    float err = p->setpoint - measurement;
    p->integ += err * 0.05f; // sample dt ~50ms used in main
    float deriv = (err - p->last) / 0.05f;
    p->last = err;
    float out = p->kp * err + p->ki * p->integ + p->kd * deriv;
    if (out < p->out_min) out = p->out_min;
    if (out > p->out_max) out = p->out_max;
    return out;
}