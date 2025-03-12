#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <configandpins.h>

class Drive_system {
public:
    Drive_system() {};
    void init();
    void run(float speed, float angle, float heading, float correction, float batteryLevel);
private: 
    int motorInA[MOTORNUM] = {FRINA, BRINA, BLINA, FLINA};
    int motorInB[MOTORNUM] = {FRINB, BRINB, BLINB, FLINB};
    int motorPWM[MOTORNUM] = {FRPWM, BRPWM, BLPWM, FLPWM};
    int motorAngles[MOTORNUM] = {45, 135, 225, 315};
    float values[MOTORNUM] = {0};
    float scaleFactor = 0;
};

#endif