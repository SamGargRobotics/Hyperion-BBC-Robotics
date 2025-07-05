/*!
 * @file LSLB.h
 *
 * This is a library to calculate the lines direction for outs
 *
 * T.McCabe (Brisbane Boys' College)
 * S.Garg (Brisbane Boys' College)
 */
#ifndef LSystem_H
#define LSystem_H
 
#include <Arduino.h>
#include <config.h>
#include <common.h>
#include <pins.h>
#include <math.h>
 
/*!
 * @brief Class that stores state and functions for calculating the direction of
          the line for outs
 */
class LSystem {
public:
    LSystem() {};
    void init();
    float calculateLineDirection(float rot);
    int readOne(int sensor_num);
    int lineState = 0;
    float previousLineDirections = -1;
    bool imOnLine = 0;
    int moveSpeed = 0;
    int clusterAmount = 0;
private:
    int circularConstrain(int value,int min, int max);
    void calculateLineState(float rot);
    bool case2Check(float prevDir, float rot);
    float lineDirection = -1;
    float relativeLineDirection = -1;
    int whiteThreshold[NUM_LS] = {0};
    int pinList[4] = {LIGHT_PIN_DIGI_0, LIGHT_PIN_DIGI_1, LIGHT_PIN_DIGI_2,
                     LIGHT_PIN_DIGI_3};
    int sensorValues[NUM_LS] = {0};
    bool sensorIsWhite[NUM_LS] = {0};
    int maxIndex = 44;
    int minIndex = 44;
    int clustersList[3][2];
    float clusterCenter[3] = {44, 44, 44};
    float center = 0;
    float loopTotal = 0;
    float loopCount = 0;
    float small = 0;
    float big = 0;
    float dif = 0;
    float outerAngles[2] = {0};
    float innerAngle = 0;
    float averages[2] = {0};
};
 
#endif
