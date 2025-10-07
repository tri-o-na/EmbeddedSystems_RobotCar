#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include "hardware/timer.h"
#include "hardware/gpio.h"

static const uint32_t ECHO_TIMEOUT_US = 30000;  // ~30 ms ≈ >5 m round-trip


void setupUltrasonicPins(uint trigPin, uint echoPin) {
    gpio_init(trigPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_put(trigPin, 0);

    gpio_init(echoPin);
    gpio_set_dir(echoPin, GPIO_IN);
    gpio_pull_down(echoPin);
}

static inline bool wait_for_level(uint pin, int level, uint32_t timeout_us) {
    uint64_t start = time_us_64();
    while (gpio_get(pin) != level) {
        if ((time_us_64() - start) > timeout_us) return false;
        tight_loop_contents();
    }
    return true;
}

int64_t pulse_in_us(uint trigPin, uint echoPin, uint32_t timeout_us) {
    // Ensure a clean low before trigger
    gpio_put(trigPin, 0);
    sleep_us(3);

    // 10 µs trigger pulse
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    // Wait for rising edge, with timeout
    if (!wait_for_level(echoPin, 1, timeout_us)) return -1;

    uint64_t start = time_us_64();

    // Wait for falling edge, with timeout (from start)
    while (gpio_get(echoPin) == 1) {
        if ((time_us_64() - start) > timeout_us) return -2;
        tight_loop_contents();
    }

    return (int64_t)(time_us_64() - start);
}

float get_distance_cm(uint trigPin, uint echoPin) {
    int64_t dur = pulse_in_us(trigPin, echoPin, ECHO_TIMEOUT_US);
    if (dur < 0) return -1.0f;          // error / timeout
    return (float)dur / 58.0f;          // HC-SR04(P): ~58 µs per cm (round-trip)
}

float get_distance_in(uint trigPin, uint echoPin) {
    int64_t dur = pulse_in_us(trigPin, echoPin, ECHO_TIMEOUT_US);
    if (dur < 0) return -1.0f;
    return (float)dur / 148.0f;         // ~148 µs per inch (round-trip)
}
