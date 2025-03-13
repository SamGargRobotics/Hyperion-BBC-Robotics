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

void Drive_system::run(float speed, float angle, float heading, float correction, float batteryLevel, bool moveToggle) {
    // If batteryLevel is 0 or less, stop all motors
    if (batteryLevel <= 0) {
        scaleFactor = 0.0f;  // Completely stop all motors
    } else if (batteryLevel > 100) {
        scaleFactor = 1.0f;  // Cap scaling at 100%
    } else {
        scaleFactor = batteryLevel / 100.0f;  // Scale normally
    }

    // Calculate motor values with scaling
    for (uint8_t i = 0; i < MOTORNUM; i++) {
        values[i] = (sinf(DEG_TO_RAD * (motorAngles[i] - angle)) * speed + correction) * scaleFactor;

        // If a value is negative, stop that motor
        if (values[i] < 0) {
            values[i] = 0;
        }

        // Apply values to motors
        if(moveToggle) {
            analogWrite(motorPWM[i], values[i]);
            digitalWrite(motorInA[i], values[i] > 0);
            digitalWrite(motorInB[i], values[i] < 0);
        }
    }
}
