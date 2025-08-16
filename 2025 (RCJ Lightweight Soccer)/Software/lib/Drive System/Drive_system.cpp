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
    for(uint8_t i = 0; i < MOTORNUM; i++) {
        values[i] = cosf(DEG_TO_RAD * (angle + motorAngles[i])) * speed + correction;
    }

    #if DEBUG_MOTORS
        for(uint8_t i = 0; i < MOTORNUM; i++) {
            Serial.print(values[i]);
            Serial.print("\t");
        }
        // Serial.println();
    #endif

    float largestSpeed = 0.0f;

    for(uint8_t i = 0; i < MOTORNUM; i++) {
        if(fabs(values[i]) > largestSpeed) {
            largestSpeed = fabs(values[i]);
        }
    }

    if(largestSpeed > 255.0f) {
        for(uint8_t i = 0; i < MOTORNUM; i++) {
            values[i] *= (255.0f / largestSpeed);
        }
    }
    for(uint8_t i = 0; i < MOTORNUM; i++) {
        uint8_t speed = round(fabs(values[i]));
        analogWrite(motorPWM[i], speed);
        digitalWrite(motorInA[i], (speed >= 0));
        digitalWrite(motorInB[i], (speed <= 0));
    }
}
