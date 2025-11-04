#include "motor_encoder.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <stdio.h>

#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
#define ENA 12
#define ENB 13
#define ENC_LEFT 4
#define ENC_RIGHT 6

#define WRAP_VALUE 31250
#define CLOCK_DIVIDER 16.0f

// Single speed control - adjust this to change overall speed
#define GLOBAL_MAX_SPEED 0.08f   // Match or slightly higher than base_speed

static uint sliceA, sliceB;
static volatile uint32_t left_ticks = 0;
static volatile uint32_t right_ticks = 0;

static void encoder_isr(uint gpio, uint32_t events)
{
    if (gpio == ENC_LEFT)
        left_ticks++;
    if (gpio == ENC_RIGHT)
        right_ticks++;
}

void motors_and_encoders_init(void)
{
    // PWM setup for ENA and ENB (speed control)
    gpio_set_function(ENA, GPIO_FUNC_PWM);
    gpio_set_function(ENB, GPIO_FUNC_PWM);
    sliceA = pwm_gpio_to_slice_num(ENA);
    sliceB = pwm_gpio_to_slice_num(ENB);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, CLOCK_DIVIDER);
    pwm_config_set_wrap(&cfg, WRAP_VALUE);
    pwm_init(sliceA, &cfg, true);
    pwm_init(sliceB, &cfg, true);
    pwm_set_gpio_level(ENA, 0);
    pwm_set_gpio_level(ENB, 0);

    // Direction pins
    gpio_init(IN1);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_init(IN2);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_init(IN3);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_init(IN4);
    gpio_set_dir(IN4, GPIO_OUT);

    // Start with motors stopped
    gpio_put(IN1, 0);
    gpio_put(IN2, 0);
    gpio_put(IN3, 0);
    gpio_put(IN4, 0);

    // Encoder pins
    gpio_init(ENC_LEFT);
    gpio_set_dir(ENC_LEFT, GPIO_IN);
    gpio_pull_up(ENC_LEFT);
    gpio_init(ENC_RIGHT);
    gpio_set_dir(ENC_RIGHT, GPIO_IN);
    gpio_pull_up(ENC_RIGHT);

    gpio_set_irq_enabled_with_callback(ENC_LEFT, GPIO_IRQ_EDGE_RISE, true, &encoder_isr);
    gpio_set_irq_enabled_with_callback(ENC_RIGHT, GPIO_IRQ_EDGE_RISE, true, &encoder_isr);

    printf("Motors and encoders initialized.\n");
    printf("Max speed cap: %.0f%%\n", GLOBAL_MAX_SPEED * 100.0f);
}

void motor_set(float left, float right)
{
    // Get absolute values for magnitude
    float left_mag = (left < 0) ? -left : left;
    float right_mag = (right < 0) ? -right : right;

    // Clamp to global max
    if (left_mag > GLOBAL_MAX_SPEED) left_mag = GLOBAL_MAX_SPEED;
    if (right_mag > GLOBAL_MAX_SPEED) right_mag = GLOBAL_MAX_SPEED;

    // LEFT motor direction (changed: negative = FORWARD)
    if (left < 0) {
        // Forward (negative input means forward in the existing main.c)
        gpio_put(IN1, 0);
        gpio_put(IN2, 1);
    } else if (left > 0) {
        // Backward
        gpio_put(IN1, 1);
        gpio_put(IN2, 0);
    } else {
        gpio_put(IN1, 0);
        gpio_put(IN2, 0);
    }

    // RIGHT motor direction (changed: negative = FORWARD)
    if (right < 0) {
        // Forward (negative input means forward)
        gpio_put(IN3, 1);
        gpio_put(IN4, 0);
    } else if (right > 0) {
        // Backward
        gpio_put(IN3, 0);
        gpio_put(IN4, 1);
    } else {
        gpio_put(IN3, 0);
        gpio_put(IN4, 0);
    }

    // Set PWM speed
    pwm_set_gpio_level(ENA, (uint16_t)(left_mag * WRAP_VALUE));
    pwm_set_gpio_level(ENB, (uint16_t)(right_mag * WRAP_VALUE));
}

void motors_stop(void)
{
    pwm_set_gpio_level(ENA, 0);
    pwm_set_gpio_level(ENB, 0);
    gpio_put(IN1, 0);
    gpio_put(IN2, 0);
    gpio_put(IN3, 0);
    gpio_put(IN4, 0);
}

uint32_t encoder_pulse_width_us(int motor_index)
{
    return (motor_index == 1) ? left_ticks : (motor_index == 2) ? right_ticks : 0;
}

uint32_t encoder_get_count(int motor_index)
{
    if (motor_index == 1) return left_ticks;
    if (motor_index == 2) return right_ticks;
    return 0;
}

void encoder_reset_counts(void)
{
    left_ticks = 0;
    right_ticks = 0;
}

void encoder_release_pin_for_servo(void) {
    gpio_set_irq_enabled(15, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    printf("DBG: Released GPIO 15 for servo\n");
}

void encoder_reclaim_pin_from_servo(void) {
    gpio_set_function(15, GPIO_FUNC_SIO);
    gpio_set_dir(15, GPIO_IN);
    gpio_pull_up(15);
    gpio_set_irq_enabled(15, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    printf("DBG: Reclaimed GPIO 15 for encoder\n");
}