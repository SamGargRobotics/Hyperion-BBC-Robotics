/*!
 * @file drive_system.h
 * 
 * This is a library that calculates the speed of the 4 motors based on a
 * singular direction that the robot should move in.
 * 
 * S.Garg (Brisbane Boys' College)
 */
#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <config.h>
#include <common.h>
#include <pins.h>
#include <math.h>

/*!
 * @brief Class that stores state and functions for calculating the direction
         relative to speed for 4 motors.
 */

class Drive_system {
public:
    Drive_system() {};
    void init();
    void run(float speed, float angle, float correction);
    bool attack = true;
private: 
    int motorInA[MOTORNUM] = {FRINA, FLINA, BLINA, BRINA};
    int motorInB[MOTORNUM] = {FRINB, FLINB, BLINB, BRINB};
    int motorPWM[MOTORNUM] = {FRPWM, FLPWM, BLPWM, BRPWM};
    int motorAngles[MOTORNUM] = {45, 135, 225, 315};
    float values[MOTORNUM] = {0};
    float largestSpeed = 0;
    float constrainFactor = 0;
    float scaleFactor = 0;
    float moveCalcDir = -1;
};

#endif