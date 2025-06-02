/*!
 * @file DirCalc.h
 * 
 * @mainpage This is a library to calculate movement directions and general calculations 
 * in terms of robot movement
 * 
 * @date 30/04/25
 * 
 * @author S.Garg (Brisbane Boys' College)
 */
#ifndef DIRCALC_H
#define DIRCALC_H

#include <Arduino.h>
#include <common.h>
#include <config.h>
#include <common.h>
#include <math.h>

/*!
 * @brief Class that stores state and functions for calculating orbit and 
 *        movement directions
 */
class DirectionCalc {
public:
    DirectionCalc() {};
    float exponentialOrbit(float ballDir, float ballStr);
    float calcSpeed(float ballStr);
    float ballDis = 0;
    float defenderRotationOffset = 0;
    bool attack = false;
private:
    double ballStr_max = 0;
    double ballStr_actual = 0;
    double speed = 0;
    int StandardCases[8] = {0, 45, 90, 135, 180, 225, 270, 315};
    int standardCaseNum = 0;
    float defenderMoveDir = 0;
    float mainAngle = 0;
    float moveAngle = 0;
    float secondaryAngle = 0;
    float lateralMoveDis = 0;
    float verticalMoveDis = 0;
    float modBallDir = 0;
};

#endif