// motor_encoder.c
#include "motor_encoder.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <stdio.h>

// MOTOR PINS
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11

// ENCODER PINS (signal only, assumed single-channel)
#define ENC1_A 4     // Left encoder
#define ENC2_A 6     // Right encoder

// PWM SETUP
#define PWM_WRAP    500u
#define PWM_DIVIDER 4.0f

// Encoder counters (rising edge count)
static volatile uint32_t enc1_count = 0;
static volatile uint32_t enc2_count = 0;

// ==============================
//     ENCODER INTERRUPT
// ==============================
static void encoder_isr(uint gpio, uint32_t events)
{
    if (!(events & GPIO_IRQ_EDGE_RISE)) {
        // We only care about rising edges to count pulses
        return;
    }

    if (gpio == ENC1_A) {
        enc1_count++;
    } else if (gpio == ENC2_A) {
        enc2_count++;
    }
}

// ==============================
//         PWM INIT
// ==============================
static void pwm_init_pin(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, PWM_DIVIDER);
    pwm_config_set_wrap(&cfg, PWM_WRAP);

    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(gpio, 0);
}

// convert -1 to +1 → PWM 0–wrap
static inline uint16_t duty_from_float(float s)
{
    if (s < 0.0f) s = -s;
    if (s > 1.0f) s = 1.0f;
    return (uint16_t)(s * PWM_WRAP);
}

// ==============================
//         MOTOR CONTROL
// ==============================
//
// Assumption: For BOTH motors, “forward” is achieved by
// driving PWM on the B pin and keeping A low.
// Reverse is the opposite.
// If your wiring is different, just swap A/B logic
// for the affected motor.
//
void motor_set(float m1, float m2)
{
    uint16_t d1 = duty_from_float(m1);
    uint16_t d2 = duty_from_float(m2);

    // ===== MOTOR 1 (LEFT) =====
    if (m1 > 0.0f) {
        // Forward: PWM on M1B
        pwm_set_gpio_level(M1A, 0);
        pwm_set_gpio_level(M1B, d1);
    } else if (m1 < 0.0f) {
        // Reverse: PWM on M1A
        pwm_set_gpio_level(M1A, d1);
        pwm_set_gpio_level(M1B, 0);
    } else {
        pwm_set_gpio_level(M1A, 0);
        pwm_set_gpio_level(M1B, 0);
    }

    // ===== MOTOR 2 (RIGHT) =====
    if (m2 > 0.0f) {
        // Forward: PWM on M2B (your observation)
        pwm_set_gpio_level(M2A, 0);
        pwm_set_gpio_level(M2B, d2);
    } else if (m2 < 0.0f) {
        // Reverse: PWM on M2A
        pwm_set_gpio_level(M2A, d2);
        pwm_set_gpio_level(M2B, 0);
    } else {
        pwm_set_gpio_level(M2A, 0);
        pwm_set_gpio_level(M2B, 0);
    }
}

// ==============================
//          STOP
// ==============================
void motors_stop(void)
{
    motor_set(0.0f, 0.0f);
}

// ==============================
//          INIT
// ==============================
void motors_and_encoders_init(void)
{
    // PWM motor pins
    pwm_init_pin(M1A);
    pwm_init_pin(M1B);
    pwm_init_pin(M2A);
    pwm_init_pin(M2B);

    // Encoder pins
    gpio_init(ENC1_A);
    gpio_set_dir(ENC1_A, GPIO_IN);
    gpio_pull_up(ENC1_A);

    gpio_init(ENC2_A);
    gpio_set_dir(ENC2_A, GPIO_IN);
    gpio_pull_up(ENC2_A);

    // Interrupt: attach common callback to ENC1_A,
    // then enable both encoders on rising edge only.
    gpio_set_irq_enabled_with_callback(
        ENC1_A,
        GPIO_IRQ_EDGE_RISE,
        true,
        &encoder_isr
    );

    gpio_set_irq_enabled(
        ENC2_A,
        GPIO_IRQ_EDGE_RISE,
        true
    );

    encoder_reset_counts();

    printf("Motors + Encoders initialized.\n");
}

// ==============================
//        ENCODER READING
// ==============================
void encoder_get_counts(uint32_t *left, uint32_t *right)
{
    if (left)  *left  = enc1_count;
    if (right) *right = enc2_count;
}

void encoder_reset_counts(void)
{
    enc1_count = 0;
    enc2_count = 0;
}
