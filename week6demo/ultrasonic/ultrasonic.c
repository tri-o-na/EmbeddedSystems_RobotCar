#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>  // Add this for qsort
#include <math.h>    // Add this for fabsf
#include "hardware/timer.h"
#include "ultrasonic.h"

static const uint32_t ECHO_TIMEOUT_US = 50000;  // ~30 ms ≈ >5 m round-trip
static float last_valid_distance = 0;
static const float MAX_DETECTION_RANGE_CM = 50.0f;  // Maximum detection range

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
    sleep_us(5);  // Increased for better stability

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

// Modified to return -1 if distance > 20cm (no detection)
float get_distance_cm(uint trigPin, uint echoPin) {
    int64_t dur = pulse_in_us(trigPin, echoPin, ECHO_TIMEOUT_US);
    if (dur < 0) return -1.0f;          // error / timeout
    
    // More precise calculation for short distances
    // Speed of sound = 343 m/s at 20°C
    // Distance = (time * speed) / 2 (round trip)
    // Distance(cm) = (time_us * 0.0343) / 2 = time_us * 0.01715
    float distance = (float)dur * 0.01715f;
    
    // Return -1 if object is beyond 20cm (no detection)
    if (distance > MAX_DETECTION_RANGE_CM) {
        return -1.0f;  // No object detected within range
    }
    
    return distance;
}

float get_distance_in(uint trigPin, uint echoPin) {
    int64_t dur = pulse_in_us(trigPin, echoPin, ECHO_TIMEOUT_US);
    if (dur < 0) return -1.0f;
    
    float distance_inches = (float)dur / 148.0f;
    float distance_cm = distance_inches * 2.54f;  // Convert to cm for range check
    
    // Return -1 if object is beyond 20cm
    if (distance_cm > MAX_DETECTION_RANGE_CM) {
        return -1.0f;  // No object detected within range
    }
    
    return distance_inches;
}

// Comparison function for sorting (median filter)
static int compare_floats(const void *a, const void *b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

// Modified for 20cm max range with 1-second intervals
float get_stable_distance_cm(uint trigPin, uint echoPin) {
    float readings[10];  // More samples for better accuracy
    float sum = 0;
    int valid_readings = 0;
    
    for (int i = 0; i < 10; i++) {
        float dist = get_distance_cm(trigPin, echoPin);
        
        // Only accept readings within detection range (1-20cm)
        if (dist >= 1.0 && dist <= MAX_DETECTION_RANGE_CM) {
            readings[valid_readings] = dist;
            sum += dist;
            valid_readings++;
        }
        
        sleep_ms(20); // 20ms delay between readings (total ~200ms)
    }
    
    if (valid_readings < 3) return -1;  // Need at least 3 valid readings
    
    return sum / valid_readings;
}

// Modified median filter for 20cm max range
float get_median_distance_cm(uint trigPin, uint echoPin) {
    float readings[10];  // More samples
    int valid_readings = 0;
    
    for (int i = 0; i < 10; i++) {
        float dist = get_distance_cm(trigPin, echoPin);
        
        // Only accept readings within detection range (1-20cm)
        if (dist >= 1.0 && dist <= MAX_DETECTION_RANGE_CM) {
            readings[valid_readings] = dist;
            valid_readings++;
        }
        
        sleep_ms(20); // 20ms delay between readings
    }
    
    if (valid_readings < 5) return -1; // Need at least 5 readings for good median
    
    // Sort the valid readings
    qsort(readings, valid_readings, sizeof(float), compare_floats);
    
    // Return median
    return readings[valid_readings / 2];
}

// Legacy compatibility functions
int getCm(int trigPin, int echoPin) {
    float distance = get_stable_distance_cm((uint)trigPin, (uint)echoPin);
    return (distance > 0) ? (int)distance : -1;
}

int getInch(int trigPin, int echoPin) {
    float distance = get_distance_in((uint)trigPin, (uint)echoPin);
    return (distance > 0) ? (int)distance : -1;
}
