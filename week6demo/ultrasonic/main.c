#include "pico/stdlib.h"
#include <stdio.h>

#define TRIG_PIN 3
#define ECHO_PIN 2
#define ECHO_TIMEOUT_US 30000  // ~5m round trip

static inline bool wait_for_level(uint pin, int level, uint32_t timeout_us) {
    absolute_time_t start = get_absolute_time();
    while (gpio_get(pin) != level) {
        if (absolute_time_diff_us(start, get_absolute_time()) > timeout_us) return false;
        tight_loop_contents();
    }
    return true;
}

static int64_t pulse_in_us(uint trigPin, uint echoPin, uint32_t timeout_us) {
    // Ensure clean low before trigger
    gpio_put(trigPin, 0);
    sleep_us(3);

    // 10µs trigger
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    // wait for rise
    if (!wait_for_level(echoPin, 1, timeout_us)) return -1;
    absolute_time_t start = get_absolute_time();

    // wait for fall
    while (gpio_get(echoPin) == 1) {
        if (absolute_time_diff_us(start, get_absolute_time()) > timeout_us) return -2;
        tight_loop_contents();
    }
    return absolute_time_diff_us(start, get_absolute_time());
}

static void setup_ultrasonic(uint trig, uint echo) {
    gpio_init(trig);
    gpio_set_dir(trig, true);
    gpio_put(trig, 0);

    gpio_init(echo);
    gpio_set_dir(echo, false);
    gpio_pull_down(echo); // keeps ECHO sane on some modules
}

int main() {
    stdio_init_all();
    sleep_ms(1500);                // let USB enumerate
    printf("Ultrasonic test starting...\n");

    setup_ultrasonic(TRIG_PIN, ECHO_PIN);

    while (true) {
        int64_t dur = pulse_in_us(TRIG_PIN, ECHO_PIN, ECHO_TIMEOUT_US);
        if (dur < 0) {
            printf("No echo (err %lld)\n", dur);
        } else {
            // ~58us per cm round-trip; ~148us per inch
            float cm = dur / 58.0f;
            printf("Distance: %.1f cm (pulse=%lld us)\n", cm, dur);
        }
        fflush(stdout);
        sleep_ms(500);
    }
}
