/*!
 * @file LSLB.cpp
 *
 * @mainpage LS calculations for Robot
 *
 * This is a library for LS calculations within the robot's code.
*/
 
#include "LSLB.h"
 
/*!
 * @brief Initalizes The Light System
*/
void LSystem::init() {
    for(uint8_t i = 0; i < 4; i++) {
        pinMode(pinList[i], OUTPUT);
    }
    pinMode(LIGHT_PIN, INPUT);
    pinMode(LIGHT_PIN2, INPUT);
}
/*!
 * @brief a constraint function that loops to the other side/max or min if out side of the max and min
 * @param value the value the is being constrained
 * @param min the minim value to constrain to
 * @param max the maxium vaule to constrain to
 * @return the value after the constrains are applied
*/
int LSystem::loopReadClamp(int value, int min, int max) {
    if (value < 0 || min != 0) {
        return ((value % max) + max) % max;
    } else {
        return value % max;
    }
}
 
/*!
 * @brief reads a single light sensor
 * @param sensor_num the sensors index in the light sensor array that you want to read
 * @return returns the value of the light sensor
*/
int LSystem::readOne(int sensor_num) {
    for(int i = 0; i < 4; i++) {
        digitalWrite(pinList[i], (sensor_num>>i)&0x01);
    }    
    if((sensor_num>>4)&0x01 == 0) {
        return analogRead(LIGHT_PIN);
    } else {
        return analogRead(LIGHT_PIN2);
    }
}
 
/*!
 * @brief calculates the direction of the white line to detect outs
 * @return returns the direction of the line 0 - 31 (returns it as the light sensors index)
*/
int LSystem::calculateLineDirection() {
    int senorValues[NUM_LS] = {0};
    int senorIsWhite[NUM_LS] = {0};
    for(int i = 0; i < NUM_LS; i++) {
        senorValues[i] = readOne(i);
        if(senorValues[i] >= whiteThreshold) {
            senorIsWhite[i] = 1;
        }
    }
    int maxIndex = 44;
    int minIndex = 44;
    int clustersList[3][2];
    int clusterAmount = 0; //if three die
    for(int j = 0; j < NUM_LS; j++) {
        if(senorIsWhite[j] == 1) {
            if(senorIsWhite[loopReadClamp(j-1,0,NUM_LS)] == 0) { //loopReadClamp(j-1,0,NUM_LS)
                minIndex = j;
            } else if(senorIsWhite[loopReadClamp(j-1,0,NUM_LS)] == 0) { //loopReadClamp(j-1,0,NUM_LS)
                maxIndex = j;
                clustersList[clusterAmount][0] = minIndex;
                clustersList[clusterAmount][1] = maxIndex;
                clusterAmount += 1;
            } else if (j == 31) {
                clustersList[0][0] = minIndex;
            }
        }
    }
    int clusterCenter[3] = {44, 44, 44};
    int center;
    for(uint8_t c = 0; c < clusterAmount; c++) {
        minIndex = clustersList[c][0];
        maxIndex = clustersList[c][1];
 
        if(minIndex>maxIndex) {
            center = (loopReadClamp(maxIndex+minIndex,0,NUM_LS))/2;
            if(maxIndex+minIndex < NUM_LS) {
                center += 16;
            }
        } else {
            center = (maxIndex+minIndex)/2;
        }
        clusterCenter[c] = center;
    }
    int averageDirection;
    int loopTut = 0;
    uint8_t loopCount = 0;
    for(uint8_t k = 0; k < 4; k++) {
        if(clusterCenter[k] != 44) {
            loopTut += clusterCenter[k];
            loopCount += 1;
        }
    }
    averageDirection = round(loopTut/loopCount);
    return averageDirection;
}
 