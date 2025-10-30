#ifndef PID_H
#define PID_H
typedef struct {
    float kp, ki, kd;
    float setpoint;
    float prev_error, integral;
    float min_out, max_out;
} PID;

void pid_init(PID *pid, float kp, float ki, float kd,
              float min_out, float max_out);
float pid_update(PID *pid, float input);
#endif
