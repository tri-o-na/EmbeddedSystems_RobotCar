#ifndef ULTRASONIC_OBJECT_H
#define ULTRASONIC_OBJECT_H

class Ultrasonic
{
    private:
    int trigPin;
    int echoPin;

    public:
    Ultrasonic(int trigPin, int echoPin);
    int getCM();
    int getINCH();
    
    // Add new stable measurement methods
    float getStableCM();
    float getMedianCM();
};

#endif