/*!
 * @file DirCalc.h
 * 
 * This is a library to calculate movement directions and general calculations 
 * in terms of robot movement
 * 
 * S.Garg (Brisbane Boys' College)
*/
#ifndef DIRCALC_H
#define DIRCALC_H

#include <Arduino.h>
#include <configandpins.h>
#include <math.h>

/*!
 * @brief Class that stores state and functions for calculating orbit and 
 *        movement directions
*/
class DirectionCalc {
public:
    DirectionCalc() {};
    bool defenderMoving = false;
    bool calculateStrategy(float externalBallDis, float ballDis);
    float trigOrbit(float ballStr, float ballDir);
    float exponentialOrbit(float ballDir);
    float calcSpeed(float ballStr);
    float ballDis = 0;
    float defenderRotationOffset = 0;
    int defenderMovement(float goalDir, float goalDis, float ballDir);
    void defenderRotCalc(float goalDir);
private:
    double findMiddleAngle(double angle1, double angle2);
    int StandardCases[8] = {0, 45, 90, 135, 180, 225, 270, 315};
    int standardCaseNum = 0;
    int goalDisRelativeDirection = 0;
    int ballRobotRelativeDirection = 0;
    int goalDisCalculatedRelativeDirection = -1;
    float decreaseVerticalDis(float verticalDis);
    float increaseVerticalDis(float verDis);
    float ballDisScale(float ballStr); 
    float mainAngle = 0;
    float moveAngle = 0;
    float secondaryAngle = 0;
    float lateralMoveDis = 0;
    float verticalMoveDis = 0;
};

#endif