#include "drive_system.h"
#include <configandpins.h>
#include <math.h>

void Drive_system::init() {
    // Initialize the motor pins so that you may send data to them.
    for(uint8_t i = 0; i < MOTORNUM; i++) {
        pinMode(motorInA[i], OUTPUT);
        pinMode(motorInB[i], OUTPUT);
        pinMode(motorPWM[i], OUTPUT);
    }
}

void Drive_system::run(float speed, float angle, float heading, float correction) {
    // Complete movement calculations and assign them to the 'values' list to write to the motors for later.
    for(uint8_t i = 0; i < MOTORNUM; i++) {
        values[i] = sinf(DEG_TO_RAD * (motorAngles[i] - angle)) * speed + correction;
    }

    // Use the calculations saved in the list 'values' to write to the motors (pwm, analog), (inA/inB, digitally).
    for(uint8_t i = 0; i < MOTORNUM; i++) {
        analogWrite(motorPWM[i], values[i]);
        digitalWrite(motorInA[i], (values[i]) > 0);
        digitalWrite(motorInB[i], (values[i]) < 0);
    }
}