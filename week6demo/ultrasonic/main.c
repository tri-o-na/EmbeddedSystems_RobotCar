#include "pico/stdlib.h"
#include <stdio.h>

// Pin assignments
#define TRIG_PIN 0
#define ECHO_PIN 1
#define ECHO_TIMEOUT_US 30000  // 30 ms ≈ 5 m max distance

// Waits for the echo pin to reach a level (HIGH or LOW) with a timeout
static inline bool wait_for_level(uint pin, int level, uint32_t timeout_us) {
    absolute_time_t start = get_absolute_time();
    while (gpio_get(pin) != level) {
        if (absolute_time_diff_us(start, get_absolute_time()) > timeout_us)
            return false;  // Timed out
        tight_loop_contents();
    }
    return true;
}

// Sends a 10 µs trigger pulse and measures echo duration (in microseconds)
static int64_t pulse_in_us(uint trigPin, uint echoPin, uint32_t timeout_us) {
    // Ensure clean LOW before trigger
    gpio_put(trigPin, 0);
    sleep_us(3);

    // Send 10 µs HIGH pulse
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    // Wait for echo to go HIGH
    if (!wait_for_level(echoPin, 1, timeout_us))
        return -1;  // No rising edge detected

    absolute_time_t start = get_absolute_time();

    // Wait for echo to go LOW (end of pulse)
    while (gpio_get(echoPin) == 1) {
        if (absolute_time_diff_us(start, get_absolute_time()) > timeout_us)
            return -2;  // Timeout
        tight_loop_contents();
    }

    // Return total pulse duration in microseconds
    return absolute_time_diff_us(start, get_absolute_time());
}

// Initializes ultrasonic pins
static void setup_ultrasonic(uint trig, uint echo) {
    gpio_init(trig);
    gpio_set_dir(trig, GPIO_OUT);
    gpio_put(trig, 0);

    gpio_init(echo);
    gpio_set_dir(echo, GPIO_IN);
}

// Main program
int main() {
    stdio_init_all();
    setup_ultrasonic(TRIG_PIN, ECHO_PIN);

    printf("Ultrasonic Distance Measurement Demo\n");

    while (true) {
        int64_t pulse_time = pulse_in_us(TRIG_PIN, ECHO_PIN, ECHO_TIMEOUT_US);

        if (pulse_time > 0) {
            // Calculate distance in cm (speed of sound = 343 m/s)
            float distance = (pulse_time * 0.0343f) / 2.0f;
            printf("Distance: %.2f cm\n", distance);
        } else {
            printf("No echo detected\n");
        }

        sleep_ms(500);  // Delay before next measurement
    }
}