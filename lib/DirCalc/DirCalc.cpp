#include "DirCalc.h"

float DirectionCalc::trigOrbit(float ballStr, float ballDir) {
    // Check for any exceptions to the orbit (that wont work with the regular calculations)
    if(ballDir >= 1 && ballDir <= 44) {
        mainAngle = ballDir;
        lateralMoveDis = ballDis*sinf(mainAngle);
        verticalMoveDis = ballDis*cosf(mainAngle);
        secondaryAngle = atanf(decreaseVerticalDis(verticalMoveDis)/lateralMoveDis);
        moveAngle = 180 - (90+secondaryAngle);
    } else if(ballDir >= 46 && ballDir <= 89) {
        mainAngle = 90 - ballDir;
        lateralMoveDis = ballDis*cosf(mainAngle);
        verticalMoveDis = ballDis*sinf(mainAngle);
        secondaryAngle = atanf(decreaseVerticalDis(verticalMoveDis)/lateralMoveDis);
        moveAngle = 90 - secondaryAngle;
    } else if(ballDir >= 91 && ballDir <= 134) {
        mainAngle = ballDir - 90;
        lateralMoveDis = ballDis*cosf(mainAngle);
        verticalMoveDis = ballDis*sinf(mainAngle);
        secondaryAngle = atanf(increaseVerticalDis(verticalMoveDis)/lateralMoveDis);
        moveAngle = secondaryAngle + 90;
    } else if(ballDir >= 136 && ballDir <= 179) {
        mainAngle = 180 - ballDir;
        lateralMoveDis = ballDis*sinf(mainAngle);
        verticalMoveDis = ballDis*cosf(mainAngle);
        secondaryAngle = atanf(increaseVerticalDis(verticalMoveDis)/lateralMoveDis);
        moveAngle = 180 - secondaryAngle;
    } else if(ballDir >= 181 && ballDir <= 214) {
        mainAngle = ballDir - 180;
        lateralMoveDis = ballDis*sinf(mainAngle);
        verticalMoveDis = ballDis*cosf(mainAngle);
        secondaryAngle = atanf(increaseVerticalDis(verticalMoveDis)/lateralMoveDis);
        moveAngle = secondaryAngle + 180;
    } else if(ballDir >= 216 && ballDir <= 269) {
        mainAngle = 270 - ballDir;
        lateralMoveDis = ballDis*cosf(mainAngle);
        verticalMoveDis = ballDis*sinf(mainAngle);
        secondaryAngle = atanf(increaseVerticalDis(verticalMoveDis)/lateralMoveDis);
        moveAngle = 270 - secondaryAngle;
    } else if(ballDir >= 271 && ballDir <= 314) {
        mainAngle = ballDir - 270;
        lateralMoveDis = ballDis*cosf(mainAngle);
        verticalMoveDis = ballDis*sinf(mainAngle);
        secondaryAngle = atanf(decreaseVerticalDis(verticalMoveDis)/lateralMoveDis);
        moveAngle = secondaryAngle + 270;
    } else if(ballDir >= 316 && ballDir <= 359) {
        mainAngle = 360 - ballDir;
        lateralMoveDis = ballDis*sinf(mainAngle);
        verticalMoveDis = ballDir*cosf(mainAngle);
        secondaryAngle = atanf(decreaseVerticalDis(verticalMoveDis)/lateralMoveDis);
        moveAngle = 360 - secondaryAngle;
    } else {
        for(uint8_t i = 0; i < 8; i++) {
            if(ballDir == StandardCases[i]) {
                standardCaseNum = i;
                if(ballDir <= 135) {
                    moveAngle = StandardCases[standardCaseNum] + 10;
                } else if(ballDir >= 215) {
                    moveAngle = StandardCases[standardCaseNum] - 10;
                } else {
                    if(moveAngle <= 180) {
                        moveAngle = 135;
                    } else {
                        moveAngle = 215;
                    }
                }
            }
        }
    }
    return moveAngle;
}

float DirectionCalc::exponentialOrbit(float ballDir) {
    if(ballDir > 180) {
        return ballDir - min(0.04*pow(ORBIT_MULTIPLIER, 4.5*ballDir), EXPO_MIN_VAL);
    } else {
        return ballDir + min(0.04*pow(ORBIT_MULTIPLIER, 4.5*ballDir), EXPO_MIN_VAL);
    }
}

float DirectionCalc::calcSpeed(float ballStr) { return -1*BALL_STRENGTH_MULTIPLIER*ballStr+255; }
float DirectionCalc::ballDisScale(float ballStr) { return ballStr*BALL_DIS_MULTIPLIER; } //TUNE THE MULTIPLCATION BY THE CONSTANT
float DirectionCalc::decreaseVerticalDis(float verticalDis) { return (1 * verticalDis + 0); } //LINEAR FUNCTION - NEEDS TUNING
float DirectionCalc::increaseVerticalDis(float verDis) { return (1 * verDis + 0); } //LINEAR FUNCTION - NEEDS TUNING