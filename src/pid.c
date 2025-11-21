#include "pid.h"

void pid_init(PID *pid, float kp, float ki, float kd, float min, float max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->out_min = min;
    pid->out_max = max;
}

float pid_update(PID *pid, float current) {
    float error = pid->setpoint - current;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}
