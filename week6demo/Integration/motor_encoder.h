#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H
#include <stdint.h>
void motors_and_encoders_init(void);
void motor_set(float left, float right);
void motors_stop(void);
#endif
