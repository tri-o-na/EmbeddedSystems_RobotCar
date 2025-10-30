#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include "pico/stdlib.h"

void motors_and_encoders_init(void);
void motor_set(float left, float right);
void motors_stop(void);
uint32_t encoder_pulse_width_us(int motor_index);

#endif
