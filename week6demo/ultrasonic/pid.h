#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float setpoint;
    float integ;
    float last;
    float out_min, out_max;
} PID;

void pid_init(PID *p, float kp, float ki, float kd, float out_min, float out_max);
float pid_update(PID *p, float measurement);

#endif