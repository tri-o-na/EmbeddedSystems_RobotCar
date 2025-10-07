//
// Two-motor control with encoder pulse-width telemetry
// Motion sequence (repeats forever):
//  - Rotate clockwise 2 s
//  - Rotate counter-clockwise 2 s
//  - Drive forward 2 s
//  - Drive backward 2 s
//  - Pause 1 s
//

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "pico/time.h"

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
static void motor_set(float m1, float m2) {
    uint16_t d1 = duty_from_float(m1);
    uint16_t d2 = duty_from_float(m2);

    if (m1 > 0) { pwm_set_gpio_level(M1A, d1); pwm_set_gpio_level(M1B, 0); }
    else if (m1 < 0) { pwm_set_gpio_level(M1A, 0); pwm_set_gpio_level(M1B, d1); }
    else { pwm_set_gpio_level(M1A, 0); pwm_set_gpio_level(M1B, 0); }

    if (m2 > 0) { pwm_set_gpio_level(M2A, d2); pwm_set_gpio_level(M2B, 0); }
    else if (m2 < 0) { pwm_set_gpio_level(M2A, 0); pwm_set_gpio_level(M2B, d2); }
    else { pwm_set_gpio_level(M2A, 0); pwm_set_gpio_level(M2B, 0); }
}

static void motors_stop(void) {
    motor_set(0.0f, 0.0f);
}

// Run a motion for duration, with optional gentle ramp to show varying speeds
static void run_for_ms(float m1, float m2, uint32_t duration_ms, bool ramp) {
    const uint32_t step_ms = 100;
    uint32_t elapsed = 0;
    while (elapsed < duration_ms) {
        float phase = ramp ? ((float)elapsed / (float)duration_ms) : 1.0f;
        float scale = ramp ? (0.3f + 0.6f * phase) : 1.0f; // 30% -> 90%
        motor_set(m1 * scale, m2 * scale);
        sleep_ms(step_ms);
        elapsed += step_ms;
        printf("m1=%.2f m2=%.2f enc1_us=%lu enc2_us=%lu\n",
               m1 * scale, m2 * scale,
               (unsigned long)enc1_pulse_us, (unsigned long)enc2_pulse_us);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(300);

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

    printf("Two-motor motion sequence with encoder telemetry.\n");

    while (1) {
        // Rotate in place clockwise: left forward, right backward (2 s)
        run_for_ms(+1.0f, -1.0f, 2000, true);

        // Rotate in place counter-clockwise: left backward, right forward (2 s)
        run_for_ms(-1.0f, +1.0f, 2000, true);

        // Drive forward (2 s)
        run_for_ms(+1.0f, +1.0f, 2000, true);

        // Drive backward (2 s)
        run_for_ms(-1.0f, -1.0f, 2000, true);

        // Pause for 1 s
        motors_stop();
        sleep_ms(1000);
    }
}
