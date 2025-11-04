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
#define CLOCK_DIVIDER 4.0f

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
    pwm_set_gpio_level(ENA, 0); // Start stopped
    pwm_set_gpio_level(ENB, 0); // Start stopped

    // Direction pins - configure as GPIO outputs
    gpio_init(IN1);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_init(IN2);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_init(IN3);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_init(IN4);
    gpio_set_dir(IN4, GPIO_OUT);

    // CRITICAL: Set ALL direction pins to LOW for BRAKE MODE on startup
    // This prevents any motor movement until motor_set() is called
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
    printf("Direction pins set to BRAKE MODE (all LOW)\n");
    printf("Speed controlled via PWM on ENA (pin %d) and ENB (pin %d)\n", ENA, ENB);
    printf("Motors will NOT move until motor_set() is called\n");
}

void motor_set(float left, float right)
{
    // Clamp values to [-1, 1] for bidirectional control
    if (left < -1.0f)
        left = -1.0f;
    if (left > 1.0f)
        left = 1.0f;
    if (right < -1.0f)
        right = -1.0f;
    if (right > 1.0f)
        right = 1.0f;

    // Handle direction based on sign
    float left_speed = (left < 0) ? -left : left;
    float right_speed = (right < 0) ? -right : right;
    
    // Debug output
    printf("[MOTOR] Set: L=%.2f R=%.2f | Speed: L=%.2f R=%.2f\n", left, right, left_speed, right_speed);
    
    // Set direction pins based on sign
    // Left motor: IN1=LOW, IN2=HIGH = forward | IN1=HIGH, IN2=LOW = backward
    if (left >= 0) {
        gpio_put(IN1, 0);
        gpio_put(IN2, 1);  // Forward
        printf("[MOTOR] Left motor: FORWARD\n");
    } else {
        gpio_put(IN1, 1);
        gpio_put(IN2, 0);  // Backward
        printf("[MOTOR] Left motor: BACKWARD\n");
    }
    
    // Right motor: IN3=HIGH, IN4=LOW = forward | IN3=LOW, IN4=HIGH = backward
    if (right >= 0) {
        gpio_put(IN3, 1);
        gpio_put(IN4, 0);  // Forward
        printf("[MOTOR] Right motor: FORWARD\n");
    } else {
        gpio_put(IN3, 0);
        gpio_put(IN4, 1);  // Backward
        printf("[MOTOR] Right motor: BACKWARD\n");
    }
    
    // Control speed via PWM duty cycle (0 to 1)
    uint16_t left_pwm = (uint16_t)(left_speed * WRAP_VALUE);
    uint16_t right_pwm = (uint16_t)(right_speed * WRAP_VALUE);
    pwm_set_gpio_level(ENA, left_pwm);
    pwm_set_gpio_level(ENB, right_pwm);
    
    printf("[MOTOR] PWM levels: ENA=%u ENB=%u (max=%u)\n", left_pwm, right_pwm, WRAP_VALUE);
}

void motors_stop(void)
{
    // Stop motors by setting PWM to 0
    pwm_set_gpio_level(ENA, 0);
    pwm_set_gpio_level(ENB, 0);

    // Set direction pins to brake mode for complete stop
    gpio_put(IN1, 0);
    gpio_put(IN2, 0);
    gpio_put(IN3, 0);
    gpio_put(IN4, 0);
}

// Legacy function (kept for compatibility)
uint32_t encoder_pulse_width_us(int motor_index)
{
    return (motor_index == 1) ? left_ticks : (motor_index == 2) ? right_ticks
                                                                : 0;
}

// Get accumulated encoder count
uint32_t encoder_get_count(int motor_index)
{
    if (motor_index == 1)
        return left_ticks;
    if (motor_index == 2)
        return right_ticks;
    return 0;
}

// Reset encoder counts
void encoder_reset_counts(void)
{
    left_ticks = 0;
    right_ticks = 0;
}