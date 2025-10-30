#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float setpoint;
    float integral;
    float prev_error;
    float out_min, out_max;
} PID;

void pid_init(PID *pid, float kp, float ki, float kd, float min, float max);
float pid_update(PID *pid, float current);

#endif
