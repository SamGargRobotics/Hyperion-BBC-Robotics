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
    void calculateLineDirection();
    void calculateLineState();
    int lineDirection = -1;
    int lineState = 1;
    int previousLineDirections[10] = {0};
private:
    int readOne(int sensor_num);
    int loopReadClamp(int value,int min, int max);
    const int whiteThreshold = 20;
    int pinList[4] = {LIGHT_PIN_DIGI_0, LIGHT_PIN_DIGI_1, LIGHT_PIN_DIGI_2, 
                     LIGHT_PIN_DIGI_3};
    int sensorValues[NUM_LS] = {0};
    int sensorIsWhite[NUM_LS] = {0};
    int maxIndex = 44;
    int minIndex = 44;
    int clustersList[3][2];
    int clusterAmount = 0; //if three die
    int clusterCenter[3] = {44, 44, 44};
    int center;
    int loopTut = 0;
    int flaggedLineDirection = 0;
    int lineDirectionArray = 0;
    uint8_t loopCount = 0;
};
 
#endif