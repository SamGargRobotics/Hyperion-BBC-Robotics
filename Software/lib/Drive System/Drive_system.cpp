/*!
 * @file drive_system.cpp
 * 
 * @mainpage Direction : 4 motor speeds.
 * 
 * This is a library to calculate the speed of 4 motors based on the direction
 * given.
 */
#include "Drive_system.h"
#include <math.h>

/*!
 * @brief Initializes the motors for use.
 */
void Drive_system::init() {
    for(uint8_t i = 0; i < MOTORNUM; i++) {
        pinMode(motorInA[i], OUTPUT);
        pinMode(motorInB[i], OUTPUT);
        pinMode(motorPWM[i], OUTPUT);
    }
}

/*!
 * @brief Calculates the motor speed and writes to each motor accordingly.
 * 
 * @param speed Speed robot is running at.
 * @param angle Angle that the robot should move in.
 * @param correction Rotation needed to ensure that the robot stays forward.
 */
void Drive_system::run(float speed, float angle, float correction) {
    float dir = 450.0f - angle;
    for(uint8_t i = 0; i < MOTORNUM; i++) {
        values[i] = cosf(DEG_TO_RAD * (dir + 270.0 - motorAngles[i])) * speed + correction;
    }

    largestSpeed = 0;

    for(uint8_t i = 0; i < MOTORNUM; i++) {
        if(abs(values[i]) > largestSpeed) {
            largestSpeed = abs(values[i]);
        }
    }

    if(largestSpeed > 255.0f) {
        for(uint8_t i = 0; i < MOTORNUM; i++) {
            values[i] *= (255.0f / largestSpeed);
        }
    }

    #if DEBUG_MOTORS
        for(uint8_t i = 0; i < MOTORNUM; i++) {
            Serial.print(values[i]);
            Serial.print(" ");
        }
        Serial.println();
    #endif

    for(uint8_t i = 0; i < MOTORNUM; i++) {
        analogWrite(motorPWM[i], constrain(abs(round(values[i])), 0, 255));
        digitalWrite(motorInA[i], (values[i] > 0));
        digitalWrite(motorInB[i], (values[i] < 0));
    }
}