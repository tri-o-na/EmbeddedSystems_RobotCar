#ifndef ultrasonic_h
#define ultrasonic_h

#include <stdint.h>
#include <stdbool.h>  // Required for bool type
#include "pico/stdlib.h"  // Required for uint type

// Function declarations
void setupUltrasonicPins(uint trigPin, uint echoPin);
int64_t pulse_in_us(uint trigPin, uint echoPin, uint32_t timeout_us);
float get_distance_cm(uint trigPin, uint echoPin);
float get_distance_in(uint trigPin, uint echoPin);

// Stable measurement functions
float get_stable_distance_cm(uint trigPin, uint echoPin);
float get_median_distance_cm(uint trigPin, uint echoPin);

// Legacy functions for compatibility
int getCm(int trigPin, int echoPin);
int getInch(int trigPin, int echoPin);

#endif