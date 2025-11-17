#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>

void motors_and_encoders_init(void);
void motor_set(float left, float right);
void motors_stop(void);

uint32_t encoder_pulse_width_us(int motor_index);

// Read total encoder counts since startup
void encoder_get_counts(uint32_t *left, uint32_t *right);

// Reset encoder counters to zero
void encoder_reset_counts(void);
#endif