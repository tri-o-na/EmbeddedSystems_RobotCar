#include "pid.h"
#include "pico/stdlib.h"   // for time functions

void pid_init(PID *pid,
              float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->setpoint = 0.0f;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;

    // NEW FEATURES
    pid->last_derivative = 0.0f;
    pid->derivative_alpha = 0.1f;   // 0.1 = smooth, 0.5 = sharper

    pid->integral_limit = 2000.0f;  // anti-windup clamp

    pid->prev_time = to_ms_since_boot(get_absolute_time());
}

float pid_update(PID *pid, float current)
{
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    float dt = (now_ms - pid->prev_time) / 1000.0f;
    pid->prev_time = now_ms;
    if (dt <= 0.00001f) dt = 0.00001f;   // prevent div-by-zero

    // Calculate error
    float error = pid->setpoint - current;

    // --- INTEGRAL TERM WITH ANTI-WINDUP ---
    pid->integral += error * dt;

    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;

    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    // --- DERIVATIVE TERM WITH LOW-PASS FILTER ---
    float derivative_raw = (error - pid->prev_error) / dt;

    float derivative_filtered =
        pid->last_derivative +
        (pid->derivative_alpha * (derivative_raw - pid->last_derivative));

    pid->last_derivative = derivative_filtered;
    pid->prev_error = error;

    // --- PID OUTPUT ---
    float output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative_filtered;

    // Clamp
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}
