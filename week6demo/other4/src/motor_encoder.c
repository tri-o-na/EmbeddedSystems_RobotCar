#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "pico/time.h"

// --- Pin map (change to your wiring) ---
#define PIN_PWM      16   // PWMA -> enable/speed
#define PIN_DIR1     17   // IN1
#define PIN_DIR2     18   // IN2
#define PIN_ENCODER  14   // encoder signal (single channel)

// Globals for pulse width (microseconds)
static volatile uint32_t last_edge_us = 0;
static volatile uint32_t pulse_width_us = 0;

static void encoder_isr(uint gpio, uint32_t events) {
    uint32_t now = to_us_since_boot(get_absolute_time());
    if (events & GPIO_IRQ_EDGE_RISE) {
        last_edge_us = now;
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        if (last_edge_us) {
            pulse_width_us = now - last_edge_us;
        }
    }
}

static void motor_set_speed(float s) {
    // s ∈ [-1.0, 1.0]
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;

    if (s > 0) { gpio_put(PIN_DIR1, 1); gpio_put(PIN_DIR2, 0); }
    else if (s < 0) { gpio_put(PIN_DIR1, 0); gpio_put(PIN_DIR2, 1); }
    else { gpio_put(PIN_DIR1, 0); gpio_put(PIN_DIR2, 0); }

    uint16_t duty = (uint16_t)(65535.0f * (s >= 0 ? s : -s));
    uint slice = pwm_gpio_to_slice_num(PIN_PWM);
    pwm_set_gpio_level(PIN_PWM, duty);
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    // Direction pins
    gpio_init(PIN_DIR1); gpio_set_dir(PIN_DIR1, GPIO_OUT);
    gpio_init(PIN_DIR2); gpio_set_dir(PIN_DIR2, GPIO_OUT);
    gpio_put(PIN_DIR1, 0); gpio_put(PIN_DIR2, 0);

    // PWM pin
    gpio_set_function(PIN_PWM, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_PWM);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 4.0f);         // base clk / 4
    pwm_config_set_wrap(&cfg, 65535);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(PIN_PWM, 0);

    // Encoder input with IRQ on both edges
    gpio_init(PIN_ENCODER);
    gpio_set_dir(PIN_ENCODER, GPIO_IN);
    gpio_pull_up(PIN_ENCODER);
    gpio_set_irq_enabled_with_callback(PIN_ENCODER,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_isr);

    printf("Motor PWM/dir + encoder pulse width demo\n");

    // Ramp CW (0→1), then CCW (1→0)
    for (int i = 0; i <= 10; ++i) {
        motor_set_speed(i/10.0f);
        sleep_ms(300);
        printf("CW speed=%.1f pulse_width_us=%lu\n", i/10.0f, (unsigned long)pulse_width_us);
    }
    for (int i = 10; i >= 0; --i) {
        motor_set_speed(-(i/10.0f));
        sleep_ms(300);
        printf("CCW speed=-%.1f pulse_width_us=%lu\n", i/10.0f, (unsigned long)pulse_width_us);
    }
    motor_set_speed(0);
    printf("Done.\n");

    while (true) { tight_loop_contents(); }
}
