#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

// --- I2C / IMU (MPU6050) ---
// From accel_raw_mpu6050.c
#define I2C_PORT i2c0
#define PIN_SDA  4
#define PIN_SCL  5
#define MPU_ADDR 0x68

void imu_init(void);
void imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az);

// --- Ultrasonic (HC-SR04) ---
// Moved to 12/13 to avoid conflict with Motor 2 (10/11)
// Please ensure you wire the HC-SR04 to GP12 (TRIG) and GP13 (ECHO).
#define PIN_TRIG 12
#define PIN_ECHO 13

void ultrasonic_init(void);
int32_t measure_echo_us(void);

// --- Motors and Encoders ---
// From motor_encoder.c - CONFIRMED PINS
// Motor 1 (left):
#define M1A      8    // PWM A
#define M1B      9    // PWM B
#define ENC1_A   14   // Encoder 1 (Left)

// Motor 2 (right):
#define M2A      10   // PWM A
#define M2B      11   // PWM B
#define ENC2_A   15   // Encoder 2 (Right)

// PWM configuration:
#define PWM_WRAP        1000u
#define PWM_DIVIDER     6.25f

void motors_and_encoders_init(void);
void motor_set(float m1, float m2);
void motors_stop(void);
uint32_t encoder_pulse_width_us(int motor_index); // 1 or 2

// --- IR Line Sensors ---
// From ir_line_sensor.c
#define A_ADC_INPUT  0      // ADC0 -> GP26
#define A_PIN_DO     27     // DO on GP27
#define B_ADC_INPUT  2      // ADC2 -> GP28
#define B_PIN_DO     3      // DO on GP3

void ir_sensors_init(void);
// Add declarations for your ADC reading and pulse width fetching functions here...

#endif