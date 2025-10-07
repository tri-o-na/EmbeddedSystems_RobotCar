#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/time.h"

// --- Pins (adjust) ---
#define PIN_TRIG 10
#define PIN_ECHO 11

// Wait for a pin to reach a level with timeout in us
static bool wait_for_level(uint pin, bool level, uint32_t timeout_us) {
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (gpio_get(pin) != (int)level) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return false;
        }
        tight_loop_contents();
    }
    return true;
}

static int32_t measure_echo_us(void) {
    // ensure trigger low
    gpio_put(PIN_TRIG, 0);
    sleep_us(2);
    // 10 us pulse
    gpio_put(PIN_TRIG, 1);
    sleep_us(10);
    gpio_put(PIN_TRIG, 0);

    // Wait for echo rising (timeout 30 ms)
    if (!wait_for_level(PIN_ECHO, true, 30000)) return -1;
    absolute_time_t start = get_absolute_time();

    // Wait for echo falling (timeout 30 ms)
    if (!wait_for_level(PIN_ECHO, false, 30000)) return -1;
    absolute_time_t end = get_absolute_time();

    return (int32_t)absolute_time_diff_us(start, end); // positive
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    gpio_init(PIN_TRIG); gpio_set_dir(PIN_TRIG, GPIO_OUT); gpio_put(PIN_TRIG, 0);
    gpio_init(PIN_ECHO); gpio_set_dir(PIN_ECHO, GPIO_IN);

    printf("HC-SR04 distance demo\n");
    while (true) {
        int32_t us = measure_echo_us();
        if (us < 0) {
            printf("no echo\n");
        } else {
            float cm = (us * 0.0343f) * 0.5f;
            printf("echo_us=%ld  dist_cm=%.1f\n", (long)us, cm);
        }
        sleep_ms(200);
    }
}
