/*!
 * @file drive_system.cpp
 * 
 * @mainpage Direction : 4 motor speeds.
 * 
 * This is a library to calculate the speed of 4 motors based on the direction
 * given.
*/
#include "drive_system.h"
#include <configandpins.h>
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
 * @param speed Scaled speed needed.
 * @param angle Angle that the robot should move in.
 * @param heading Heading of the robot relative to the field rather than the
 *                robot direction.
 * @param correction Rotation needed to ensure that the robot stays forward.
 * @param batteryLevel Current battery level of the robot.
 * @param lineDir Direction of line (if there is not any line; -1)
 * @param goalDir Direction of attacking/defending goal depending on strategic method.
 * @param moveToggle If the robot should move or not.
*/
void Drive_system::run(float speed, float angle, float heading, float correction, float batteryLevel, float lineDir, float goalDir, bool moveToggle) {
    // If batteryLevel is 0 or less, stop all motors
    if (batteryLevel <= 0) {
        scaleFactor = 0.0f;  // Completely stop all motors
    } else if (batteryLevel > 100) {
        scaleFactor = 1.0f;  // Cap scaling at 100%
    } else {
        scaleFactor = batteryLevel / 100.0f;  // Scale normally
    }

    if(lineDir == -1) {
        moveCalcDir = angle;
    } else {
        moveCalcDir = lineDir;
    }

    // Calculate motor values with scaling
    for (uint8_t i = 0; i < MOTORNUM; i++) {
        values[i] = (sinf(DEG_TO_RAD * (motorAngles[i] - moveCalcDir)) * speed + correction + goalDir) * scaleFactor;

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