#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include "motor_encoder.h"
#include "hardware/gpio.h"

// ==== Pin map (adjust to your wiring) ====
// Motor 1 (left) on Cytron Robo Pico: sign-magnitude (two PWM pins)
#define M1A      8    // PWM A
#define M1B      9    // PWM B

// Motor 2 (right) on Cytron Robo Pico: sign-magnitude (two PWM pins)
#define M2A      10   // PWM A
#define M2B      11   // PWM B

// Encoders (single channel A per wheel)
#define ENC1_A   14
#define ENC2_A   15

// PWM configuration: ~20 kHz for quiet operation
#define PWM_WRAP        1000u
#define PWM_DIVIDER     6.25f

// Encoder pulse width storage (microseconds)
static volatile uint32_t enc1_last_rise_us = 0;
static volatile uint32_t enc2_last_rise_us = 0;
static volatile uint32_t enc1_pulse_us = 0;
static volatile uint32_t enc2_pulse_us = 0;

// IRQ callback shared by both encoder channels (A only)
static void encoder_isr(uint gpio, uint32_t events) {
    uint32_t now = to_us_since_boot(get_absolute_time());
    if (gpio == ENC1_A) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            enc1_last_rise_us = now;
        } else if ((events & GPIO_IRQ_EDGE_FALL) && enc1_last_rise_us) {
            enc1_pulse_us = now - enc1_last_rise_us;
        }
    } else if (gpio == ENC2_A) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            enc2_last_rise_us = now;
        } else if ((events & GPIO_IRQ_EDGE_FALL) && enc2_last_rise_us) {
            enc2_pulse_us = now - enc2_last_rise_us;
        }
    }
}

// Initialize one PWM output with common configuration
static void pwm_init_pin(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, PWM_DIVIDER);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(gpio, 0);
}

// Convert speed fraction [-1,1] to PWM duty [0, PWM_WRAP]
static inline uint16_t duty_from_float(float s) {
    if (s < 0.0f) s = -s;
    if (s > 1.0f) s = 1.0f;
    return (uint16_t)(s * PWM_WRAP);
}

// Set both motors' speed using sign-magnitude control on A/B pins
void motor_set(float m1, float m2) {
    printf("DBG: motor_set called with L=%.2f R=%.2f\n", m1, m2);
    // Invert the right motor if it spins the wrong way
    m2 = -m2;

    uint16_t d1 = duty_from_float(m1);
    uint16_t d2 = duty_from_float(m2);

    if (m1 > 0) { pwm_set_gpio_level(M1A, d1); pwm_set_gpio_level(M1B, 0); }
    else if (m1 < 0) { pwm_set_gpio_level(M1A, 0); pwm_set_gpio_level(M1B, d1); }
    else { pwm_set_gpio_level(M1A, 0); pwm_set_gpio_level(M1B, 0); }

    if (m2 > 0) { pwm_set_gpio_level(M2A, d2); pwm_set_gpio_level(M2B, 0); }
    else if (m2 < 0) { pwm_set_gpio_level(M2A, 0); pwm_set_gpio_level(M2B, d2); }
    else { pwm_set_gpio_level(M2A, 0); pwm_set_gpio_level(M2B, 0); }
}

void motors_stop(void) {
    printf("DBG: motors_stop called\n");
    motor_set(0.0f, 0.0f);
}

void drive_motors_stop(void) {
    motor_set(0.0f, 0.0f);
    printf("MOTORS STOPPED\n");
}

void drive_motors_forward(float speed) {
    motor_set(speed, speed);
    printf("DRIVING FORWARD at %.2f speed\n", speed);
}

void drive_motors_backward(float speed) {
    motor_set(-speed, -speed);
    printf("DRIVING BACKWARD at %.2f speed\n", speed);
}

void drive_motors_turn_left(float speed) {
    motor_set(-speed, speed);  // Left backward, right forward
    printf("TURNING LEFT at %.2f speed\n", speed);
}

void drive_motors_turn_right(float speed) {
    motor_set(speed, -speed);  // Left forward, right backward
    printf("TURNING RIGHT at %.2f speed\n", speed);
}

void drive_motors_init(void) {
    // PWM outputs on A/B pins for each motor
    pwm_init_pin(M1A);
    pwm_init_pin(M1B);
    pwm_init_pin(M2A);
    pwm_init_pin(M2B);

    // Encoder inputs on channel A, IRQ on both edges to measure pulse width
    gpio_init(ENC1_A); gpio_set_dir(ENC1_A, GPIO_IN); gpio_pull_up(ENC1_A);
    gpio_init(ENC2_A); gpio_set_dir(ENC2_A, GPIO_IN); gpio_pull_up(ENC2_A);
    gpio_set_irq_enabled_with_callback(ENC1_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_isr);
    gpio_set_irq_enabled(ENC2_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    printf("Drive motors initialized\n");
}

void drive_motors_print_telemetry(void) {
    printf("Encoder telemetry: enc1_us=%lu enc2_us=%lu\n",
           (unsigned long)enc1_pulse_us, (unsigned long)enc2_pulse_us);
}

// Compatibility wrapper: some code calls motors_and_encoders_init()
void motors_and_encoders_init(void) {
    drive_motors_init();
}

// REMOVE EVERYTHING FROM HERE DOWN - NO main() function in this file!

uint32_t encoder_pulse_width_us(int motor_index) {
    if (motor_index == 1) return enc1_pulse_us;
    if (motor_index == 2) return enc2_pulse_us;
    return 0;
}

#define ENCODER_SHARED_PIN 15

void encoder_release_pin_for_servo(void) {
    // Disable encoder IRQ on the shared pin
    gpio_set_irq_enabled(ENCODER_SHARED_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
}

void encoder_reclaim_pin_from_servo(void) {
    // Reconfigure pin as input for encoder and re-enable IRQ
    gpio_init(ENCODER_SHARED_PIN);
    gpio_set_dir(ENCODER_SHARED_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_SHARED_PIN);
    gpio_set_irq_enabled(ENCODER_SHARED_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// Obstacle detection range
#define OBSTACLE_DETECTION_RANGE 50

void drive_motors_obstacle_detection(int center_distance) {
    if (center_distance > 0 && center_distance <= OBSTACLE_DETECTION_RANGE) {
        motors_stop();
        // ... scanning and bypass logic ...
    } else {
        motor_set(-0.6f, -0.6f);
    }
}