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

#define WRAP_VALUE   31250
#define CLOCK_DIVIDER 200.0f

static uint sliceA, sliceB;
static volatile uint32_t left_ticks = 0;
static volatile uint32_t right_ticks = 0;

static void encoder_isr(uint gpio, uint32_t events) {
    if (gpio == ENC_LEFT)  left_ticks++;
    if (gpio == ENC_RIGHT) right_ticks++;
}

void motors_and_encoders_init(void) {
    // PWM setup
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
    gpio_init(IN1); gpio_set_dir(IN1, GPIO_OUT);
    gpio_init(IN2); gpio_set_dir(IN2, GPIO_OUT);
    gpio_init(IN3); gpio_set_dir(IN3, GPIO_OUT);
    gpio_init(IN4); gpio_set_dir(IN4, GPIO_OUT);
    motors_stop();

    // Encoder pins
    gpio_init(ENC_LEFT);  gpio_set_dir(ENC_LEFT, GPIO_IN);  gpio_pull_up(ENC_LEFT);
    gpio_init(ENC_RIGHT); gpio_set_dir(ENC_RIGHT, GPIO_IN); gpio_pull_up(ENC_RIGHT);

    gpio_set_irq_enabled_with_callback(ENC_LEFT, GPIO_IRQ_EDGE_RISE, true, &encoder_isr);
    gpio_set_irq_enabled(ENC_RIGHT, GPIO_IRQ_EDGE_RISE, true);

    printf("Motors and encoders initialized.\n");
}

void motor_set(float left, float right) {
    if (left < 0) left = 0;
    if (right < 0) right = 0;
    if (left > 1) left = 1;
    if (right > 1) right = 1;

    // Flip left motor direction (was IN1=1, IN2=0)
    gpio_put(IN1, 0);
    gpio_put(IN2, 1);

    // Keep right motor normal
    gpio_put(IN3, 1);
    gpio_put(IN4, 0);

    pwm_set_gpio_level(ENA, (uint16_t)(left * WRAP_VALUE));
    pwm_set_gpio_level(ENB, (uint16_t)(right * WRAP_VALUE));
}

void motors_stop(void) {
    gpio_put(IN1, 0); gpio_put(IN2, 0);
    gpio_put(IN3, 0); gpio_put(IN4, 0);
    pwm_set_gpio_level(ENA, 0);
    pwm_set_gpio_level(ENB, 0);
}

// Legacy function (kept for compatibility)
uint32_t encoder_pulse_width_us(int motor_index) {
    return (motor_index == 1) ? left_ticks :
           (motor_index == 2) ? right_ticks : 0;
}

// New function: Get accumulated encoder count
uint32_t encoder_get_count(int motor_index) {
    if (motor_index == 1) return left_ticks;
    if (motor_index == 2) return right_ticks;
    return 0;
}

// Optional: Reset encoder counts
void encoder_reset_counts(void) {
    left_ticks = 0;
    right_ticks = 0;
}