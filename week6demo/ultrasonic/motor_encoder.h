#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

// Initialization
void drive_motors_init(void);

// Added: compatibility API expected by other code
void motors_and_encoders_init(void);

// Motor control (sign-magnitude API implemented in motor_encoder.c)
void motor_set(float left, float right);
void drive_motors_stop(void);
void drive_motors_forward(float speed);
void drive_motors_backward(float speed);
void drive_motors_turn_left(float speed);
void drive_motors_turn_right(float speed);

unsigned long encoder_pulse_width_us(int motor);
void encoder_release_pin_for_servo(void);
void encoder_reclaim_pin_from_servo(void);

#endif // MOTOR_ENCODER_H