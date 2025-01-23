#ifndef DIRCALC_H
#define DIRCALC_H

#include <Arduino.h>
#include <configandpins.h>
#include <math.h>

class DirectionCalc {
public:
    DirectionCalc() {};
    float trigOrbit(float ballStr, float ballDir);
    float exponentialOrbit(float ballDir);
    float calcSpeed(float ballStr);
    float decreaseVerticalDis(float verticalDis);
    float increaseVerticalDis(float verDis);
    float ballDisScale(float ballStr); 
    float ballDis = 0;
private:
    int StandardCases[8] = {0, 45, 90, 135, 180, 225, 270, 315};
    int standardCaseNum = 0;
    float mainAngle = 0;
    float moveAngle = 0;
    float secondaryAngle = 0;
    float lateralMoveDis = 0;
    float verticalMoveDis = 0;
};

#endif