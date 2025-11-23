#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>

void motors_and_encoders_init(void);
void motor_set(float m1, float m2);
void motors_stop(void);
uint32_t encoder_pulse_width_us(int motor_index);
uint32_t encoder_get_count(int motor_index);
void encoder_reset_counts(void);

#endif
