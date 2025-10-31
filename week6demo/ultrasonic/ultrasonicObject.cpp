#include "ultrasonicObject.h"
#include "pico/stdlib.h"  // Add this include

extern "C" {
#include "ultrasonic.h"
}

Ultrasonic::Ultrasonic(int trigPinToSet, int echoPinToSet)
{
    trigPin = trigPinToSet;
    echoPin = echoPinToSet;
    setupUltrasonicPins((uint)trigPin, (uint)echoPin);
}

int Ultrasonic::getCM()
{
    // Use median filter for most stable results
    float stable_dist = get_median_distance_cm((uint)trigPin, (uint)echoPin);
    return (stable_dist > 0) ? (int)stable_dist : -1;
}

int Ultrasonic::getINCH()
{
    float cm_dist = get_stable_distance_cm((uint)trigPin, (uint)echoPin);
    if (cm_dist > 0) {
        return (int)(cm_dist / 2.54f);  // Convert cm to inches
    }
    return -1;
}

float Ultrasonic::getStableCM()
{
    return get_stable_distance_cm((uint)trigPin, (uint)echoPin);
}

float Ultrasonic::getMedianCM()
{
    return get_median_distance_cm((uint)trigPin, (uint)echoPin);
}