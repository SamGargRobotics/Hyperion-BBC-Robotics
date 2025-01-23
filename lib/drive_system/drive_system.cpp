#include "drive_system.h"
#include <configandpins.h>
#include <math.h>

void Drive_system::init() {
    for(uint8_t i = 0; i < MOTORNUM; i++) {
        pinMode(motorInA[i], OUTPUT);
        pinMode(motorInB[i], OUTPUT);
        pinMode(motorPWM[i], OUTPUT);
    }
}

void Drive_system::run(float speed, float angle, float heading, float correction) {
    for(uint8_t i = 0; i < MOTORNUM; i++) {
        values[i] = sinf(DEG_TO_RAD * (motorAngles[i] - angle)) * speed + correction;
    }

    for(uint8_t i = 0; i < MOTORNUM; i++) {
        analogWrite(motorPWM[i], values[i]);
        digitalWrite(motorInA[i], (values[i]) > 0);
        digitalWrite(motorInB[i], (values[i]) < 0);
    }
}