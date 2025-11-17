#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float setpoint;

    float integral;
    float prev_error;

    float out_min, out_max;

    // NEW
    float last_derivative;
    float derivative_alpha;   // low-pass factor (0.0 - 1.0)
    float integral_limit;     // anti-windup clamp
    float prev_time;          // for dt computation
} PID;

void pid_init(PID *pid,
              float kp, float ki, float kd,
              float out_min, float out_max);

float pid_update(PID *pid, float current);

#endif
