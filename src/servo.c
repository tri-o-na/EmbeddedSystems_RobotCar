#include "servo.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

static uint32_t servo_pin = 15;
static uint32_t servo_wrap = 20000;     // period counts ~20ms
static float servo_clkdiv = 125.0f;

void servo_set_pulse_us(uint16_t pulse_us) {
    if (!servo_pin) return;
    uint32_t level = (uint32_t)(((float)pulse_us / 20000.0f) * (float)servo_wrap);
    if (level > servo_wrap) level = servo_wrap;
    pwm_set_gpio_level((uint)servo_pin, level);
}

void servo_init(uint32_t gpio_pin) {
    servo_pin = gpio_pin;
    gpio_set_function((uint)servo_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num((uint)servo_pin);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, servo_wrap);
    pwm_config_set_clkdiv(&cfg, servo_clkdiv);
    pwm_init(slice, &cfg, true);
    servo_set_pulse_us(1500);
    sleep_ms(50);
}

void servo_move_to_center(void) {
    servo_set_pulse_us(1500);
    sleep_ms(200);
}

void servo_move_to_left(void) {
    servo_set_pulse_us(2000);
    sleep_ms(200);
}

void servo_move_to_right(void) {
    servo_set_pulse_us(1000);
    sleep_ms(200);
}

// New: slow scan implementation with adjustable delay between steps
uint16_t servo_scan_until_clear_slow(bool scan_left, float object_threshold, float (*get_distance_func)(void), uint32_t delay_ms) {
    const int step = 40; // microsecond step for servo pulse (smaller step = smoother)
    int pos;
    if (scan_left) {
        for (pos = 1500; pos <= 2000; pos += step) {
            servo_set_pulse_us((uint16_t)pos);
            sleep_ms(delay_ms);
            float d = get_distance_func();
            if (d <= 0 || d > object_threshold) return (uint16_t)pos;
        }
    } else {
        for (pos = 1500; pos >= 1000; pos -= step) {
            servo_set_pulse_us((uint16_t)pos);
            sleep_ms(delay_ms);
            float d = get_distance_func();
            if (d <= 0 || d > object_threshold) return (uint16_t)pos;
        }
    }
    return (uint16_t)pos;
}

uint16_t servo_scan_until_clear(bool scan_left, float object_threshold, float (*get_distance_func)(void)) {
    const int step = 50;
    int pos;
    if (scan_left) {
        for (pos = 1500; pos <= 2000; pos += step) {
            servo_set_pulse_us((uint16_t)pos);
            sleep_ms(80);
            float d = get_distance_func();
            if (d <= 0 || d > object_threshold) return (uint16_t)pos;
        }
    } else {
        for (pos = 1500; pos >= 1000; pos -= step) {
            servo_set_pulse_us((uint16_t)pos);
            sleep_ms(80);
            float d = get_distance_func();
            if (d <= 0 || d > object_threshold) return (uint16_t)pos;
        }
    }
    return (uint16_t)pos;
}