#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include "pico/stdlib.h"

void motors_and_encoders_init(void);
void motors_stop(void);
void encoder_release_pin_for_servo(void);
void encoder_reclaim_pin_from_servo(void);
void motor_set(float left, float right);
uint32_t encoder_pulse_width_us(int motor_index);  // Legacy
uint32_t encoder_get_count(int motor_index);       // New: get total count
void encoder_reset_counts(void);                   // Optional: reset counts

#endif