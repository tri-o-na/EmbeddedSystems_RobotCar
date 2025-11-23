#include "motor_encoder.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include <stdio.h>

#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11
#define ENC1_A 4 // Left encoder
#define ENC2_A 6 // Right encoder
#define PWM_WRAP 500u
#define PWM_DIVIDER 4.0f

// Encoder pulse timing
static volatile uint32_t enc1_last_rise_us = 0;
static volatile uint32_t enc2_last_rise_us = 0;
static volatile uint32_t enc1_pulse_us = 0;
static volatile uint32_t enc2_pulse_us = 0;

// Encoder counts
static volatile uint32_t enc1_count = 0;
static volatile uint32_t enc2_count = 0;

// ==== Encoder interrupt ====
static void encoder_isr(uint gpio, uint32_t events)
{
    uint32_t now = to_us_since_boot(get_absolute_time());
    if (gpio == ENC1_A)
    {
        if (events & GPIO_IRQ_EDGE_RISE) {
            enc1_last_rise_us = now;
            enc1_count++;
        }
        else if ((events & GPIO_IRQ_EDGE_FALL) && enc1_last_rise_us)
            enc1_pulse_us = now - enc1_last_rise_us;
    }
    else if (gpio == ENC2_A)
    {
        if (events & GPIO_IRQ_EDGE_RISE) {
            enc2_last_rise_us = now;
            enc2_count++;
        }
        else if ((events & GPIO_IRQ_EDGE_FALL) && enc2_last_rise_us)
            enc2_pulse_us = now - enc2_last_rise_us;
    }
}

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

static inline uint16_t duty_from_float(float s)
{
    if (s < 0.0f)
        s = -s;
    if (s > 1.0f)
        s = 1.0f;
    return (uint16_t)(s * PWM_WRAP);
}

void motor_set(float m1, float m2)
{
    uint16_t d1 = duty_from_float(m1);
    uint16_t d2 = duty_from_float(m2);

    if (m1 > 0)
    {
        pwm_set_gpio_level(M1A, 0);
        pwm_set_gpio_level(M1B, d1);
    }
    else if (m1 < 0)
    {
        pwm_set_gpio_level(M1A, d1);
        pwm_set_gpio_level(M1B, 0);
    }
    else
    {
        pwm_set_gpio_level(M1A, 0);
        pwm_set_gpio_level(M1B, 0);
    }

    if (m2 > 0)
    {
        pwm_set_gpio_level(M2A, d2);
        pwm_set_gpio_level(M2B, 0);
    }
    else if (m2 < 0)
    {
        pwm_set_gpio_level(M2A, 0);
        pwm_set_gpio_level(M2B, d2);
    }
    else
    {
        pwm_set_gpio_level(M2A, 0);
        pwm_set_gpio_level(M2B, 0);
    }
}

void motors_stop(void)
{
    motor_set(0.0f, 0.0f);
}

void motors_and_encoders_init(void)
{
    pwm_init_pin(M1A);
    pwm_init_pin(M1B);
    pwm_init_pin(M2A);
    pwm_init_pin(M2B);

    gpio_init(ENC1_A);
    gpio_set_dir(ENC1_A, GPIO_IN);
    gpio_pull_up(ENC1_A);
    gpio_init(ENC2_A);
    gpio_set_dir(ENC2_A, GPIO_IN);
    gpio_pull_up(ENC2_A);

    gpio_set_irq_enabled_with_callback(ENC1_A,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_isr);
    gpio_set_irq_enabled(ENC2_A,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    printf("Motors + Encoders initialized.\n");
}

uint32_t encoder_pulse_width_us(int motor_index)
{
    if (motor_index == 1)
        return enc1_pulse_us;
    if (motor_index == 2)
        return enc2_pulse_us;
    return 0;
}

uint32_t encoder_get_count(int motor_index)
{
    if (motor_index == 1) return enc1_count;
    if (motor_index == 2) return enc2_count;
    return 0;
}

void encoder_reset_counts(void)
{
    enc1_count = 0;
    enc2_count = 0;
}
