/*!
 * @file LSLB.cpp
 *
 * @mainpage LS calculations for Robot
 *
 * This is a library for LS calculations within the robot's code.
 */
 
#include "light_system.h"
 
/*!
 * @brief Initalizes The Light System
 */
void LSystem::init() {
    for(uint8_t i = 0; i < 4; i++) {
        pinMode(pinList[i], OUTPUT);
    }
    pinMode(LIGHT_PIN, INPUT);
    pinMode(LIGHT_PIN2, INPUT);

    for(int i = 0; i < 32; i++) {
        whiteThreshold[i] = readOne(i) + 40;
    }
}

/*!
 * @brief a constraint function that loops to the other side/max or min if out
          side of the max and min
 * @param value the value the is being constrained
 * @param min the minimum value to constrain to
 * @param max the maximum value to constrain to
 * @return the value after the constrains are applied
 */
int LSystem::circularConstrain(int value, int  min, int max) {
    if (value < 0 || min != 0) {
        return ((value % max) + max) % max;
    } else {
        return value % max;
    }
}
 
/*!
 * @brief reads a single light sensor
 * @param sensor_num the sensors index in the light sensor array that you want
 *                   to read
 * @return returns the value of the light sensor
 */
int LSystem::readOne(int sensor_num) {
    for(u_int8_t i = 0; i < 4; i++) {
        digitalWrite(pinList[i], (sensor_num>>i)&0x01);
    }
    if(((sensor_num>>4)&0x01) == 0) {
        return analogRead(LIGHT_PIN);
    } else {
        return analogRead(LIGHT_PIN2);
    }
}
 
/*!
 * @brief calculates the direction of the white line to detect outs
 */
float LSystem::calculateLineDirection(float rot) {
    clusterAmount = 0;
    loopTotal = 0;
    loopCount = 0;
    lineDirection = -1;
    for(int i = 0; i < 32; i++) {
        sensorIsWhite[i] = 0;
    }
    for(int i = 0; i < 3; i++) {
        clusterCenter[i] = 0;
    }
    for(int i = 0; i < NUM_LS; i++) {
        sensorValues[i] = readOne(i);
        if(sensorValues[i] >= whiteThreshold[i]) {
            sensorIsWhite[i] = 1;
        }
    }
    if(sensorIsWhite[0] && sensorIsWhite[1] && sensorIsWhite[3] && sensorIsWhite[29] && sensorIsWhite[28]) {
        // 0, 1, 3, 28, 27
        sensorIsWhite[2] = 1;
        sensorIsWhite[30] = 1;
    }
    
    for(int j = 0; j < NUM_LS; j++) {
        if(sensorIsWhite[j]) {
            if(sensorIsWhite[circularConstrain(j-1,0,NUM_LS)] == 0) {
                minIndex = j;
            }
            if(sensorIsWhite[circularConstrain(j+1,0,NUM_LS)] == 0) {
                maxIndex = j;
                clustersList[clusterAmount][0] = minIndex;
                clustersList[clusterAmount][1] = maxIndex;
                clusterAmount++;
            }
            if ((j == 31) && sensorIsWhite[0]) {
                clustersList[0][0] = minIndex;
            }
        }
    }
    for(uint8_t c = 0; c < clusterAmount; c++) {
        minIndex = clustersList[c][0];
        maxIndex = clustersList[c][1];
 
        if(minIndex>maxIndex) {
            center = (circularConstrain(maxIndex+minIndex,0,NUM_LS))/2;
            if(maxIndex+minIndex < NUM_LS) {
                center += 16;
            }
        } else {
            center = (maxIndex+minIndex)/2;
        }
        clusterCenter[c] = center * (360.0f/NUM_LS);
    }

    for(uint8_t k = 0; k < clusterAmount; k++) {
        if(clusterCenter[k] != (44*11.25)) {
            loopTotal += clusterCenter[k];
            #if DEBUG_LS
                Serial.print(" ");
                Serial.print(clusterCenter[k]);
                Serial.print(" ");
                Serial.print(clustersList[k][0]);
                Serial.print(" ");
                Serial.print(clustersList[k][1]);
            #endif
            loopCount++;
        }
    }
    #if DEBUG_LS
        Serial.print("\t");
    #endif

    #if DEBUG_LS_SENSOR
        for(int i = 0; i < NUM_LS; i++) {
            Serial.print(sensorIsWhite[i]);
            Serial.print(" ");
        }
        Serial.print("Cluster Amount: ");
        Serial.println(clusterAmount);
    #endif
    switch (clusterAmount) {
        case(0):
            lineDirection = -1;
            break;

        case(1):
            lineDirection = clusterCenter[0]; // just the 1 cluster angle
            break;
        
        case(2):
            //find angle in between 2 clusters
            small = (clusterCenter[0] > clusterCenter[1])? clusterCenter[1] : clusterCenter[0];
            big = (clusterCenter[1] > clusterCenter[0])? clusterCenter[1] : clusterCenter[0];
            dif = big - small;
            if(dif <= 180) {
                lineDirection = (big + small) / 2;
            } else {
                small += 360;
                lineDirection = (big + small) / 2;
            }
            break;
        
        case(3):
            float differences[3] = {0.00f};
            // Difference between cluster 1 and 2
            small = (clusterCenter[0] > clusterCenter[1])? clusterCenter[1] : clusterCenter[0];
            big = (clusterCenter[1] > clusterCenter[0])? clusterCenter[1] : clusterCenter[0];
            dif = big - small;
            differences[0] = (dif >= 180)?(360 - dif):dif;
            // Difference between cluster 2 and 3
            small = (clusterCenter[2] > clusterCenter[1])? clusterCenter[1] : clusterCenter[2];
            big = (clusterCenter[1] > clusterCenter[2])? clusterCenter[1] : clusterCenter[2];
            dif = big - small;
            differences[1] = (dif >= 180)?(360 - dif):dif;
            // Difference between cluster 1 and 3
            small = (clusterCenter[2] > clusterCenter[0])? clusterCenter[0] : clusterCenter[2];
            big = (clusterCenter[0] > clusterCenter[2])? clusterCenter[0] : clusterCenter[2];
            dif = big - small;
            differences[2] = (dif >= 180)?(360 - dif):dif;

            int largestDifIndex = 0;
            for(int i = 0; i < 3; i++) {
                if(differences[i] > differences[largestDifIndex]) {
                    largestDifIndex = i;
                }
            }
            if(largestDifIndex == 0) {
                outerAngles[0] = clusterCenter[0];
                outerAngles[1] = clusterCenter[1];
                innerAngle = clusterCenter[2];
            } else if(largestDifIndex == 1) {
                outerAngles[0] = clusterCenter[1];
                outerAngles[1] = clusterCenter[2];
                innerAngle = clusterCenter[0];
            } else {
                outerAngles[0] = clusterCenter[0];
                outerAngles[1] = clusterCenter[2];
                innerAngle = clusterCenter[1];
            }
            small = (outerAngles[0] > innerAngle)? innerAngle : outerAngles[0];
            big = (innerAngle > outerAngles[0])? innerAngle : outerAngles[0];
            dif = big - small;
            averages[0] = (dif <= 180)? ((big + small) / 2) : ((big + (small + 360)) / 2);

            small = (outerAngles[1] > innerAngle)? innerAngle : outerAngles[1];
            big = (innerAngle > outerAngles[1])? innerAngle : outerAngles[1];
            dif = big - small;
            averages[1] = (dif <= 180)? ((big + small) / 2) : ((big + (small + 360)) / 2);

            small = (averages[0] > averages[1])? averages[1] : averages[0];
            big = (averages[1] > averages[0])? averages[1] : averages[0];
            dif = big - small;
            lineDirection = (dif <= 180)? ((big + small) / 2) : ((big + (small + 360)) / 2);
            break;
    }
    calculateLineState(rot);
    // return lineDirection;
    switch(lineState) {
        case(0): moveSpeed = 0; return -1; break;
        case(1): moveSpeed = SET_SPEED/2; return lineDirection; break;
        case(2): moveSpeed = 3*SET_SPEED/4; return floatMod(previousLineDirections - rot, 360); break;
        case(3): moveSpeed = SET_SPEED; return floatMod(previousLineDirections - rot, 360); break;
        default:return -44; break;
    }
}

/*!
 * @brief Calculates the state at which the robot is relative to the line and
          field.
 */
void LSystem::calculateLineState(float rot) {
    imOnLine = (clusterAmount > 0);
    relativeLineDirection = imOnLine ? floatMod(lineDirection + rot, 360) : -1;
    if(lineState == 0) {
        #if DEBUG_LINE_STATE
            Serial.println("0 checking for line");
        #endif
        previousLineDirections = -1;
        if(imOnLine) {
            #if DEBUG_LINE_STATE
                Serial.println("0 into 1, enter sent");
            #endif
            lineState = 1;
            previousLineDirections = relativeLineDirection;
        }
    } else if(lineState == 1) {
        #if DEBUG_LINE_STATE
            Serial.println("1 inside and checking");
        #endif
        if(!imOnLine) {
            #if DEBUG_LINE_STATE
                Serial.println("1 into 0, enter sent");
            #endif
            lineState = 0;
        } else if((abs(previousLineDirections - relativeLineDirection) > LS_FLIP_THRESH) && (abs(previousLineDirections - relativeLineDirection) < (360 - LS_FLIP_THRESH))) {//case2Check(previousLineDirections, rot)) { // SOMETHING IS WORNG; check
            #if DEBUG_LINE_STATE
                Serial.println("1 into 2, enter sent");
            #endif
            lineState = 2;
        } else {
            #if DEBUG_LINE_STATE
                Serial.println("1 stagnated");
            #endif
            lineState = 1;
            previousLineDirections = relativeLineDirection;
        }
    } else if(lineState == 2) {
        #if DEBUG_LINE_STATE
            Serial.println("2 inside and checking");
        #endif
        if(!imOnLine) {
            #if DEBUG_LINE_STATE
                Serial.println("2 to 3, enter sent");
            #endif
            lineState = 3;
        } else if((abs(previousLineDirections - relativeLineDirection) <= LS_FLIP_THRESH) || (abs(previousLineDirections - relativeLineDirection) >= (360 - LS_FLIP_THRESH))) {//!case2Check(previousLineDirections, rot)) { // SOMETHING IS WRONG; check
            #if DEBUG_LINE_STATE
                Serial.println("2 to 1, enter sent");
            #endif
            lineState = 1;
            previousLineDirections = relativeLineDirection;
        } else {
            #if DEBUG_LINE_STATE
                Serial.println("2 stagnated");
            #endif
            lineState = 2; 
        }
    } else {
        #if DEBUG_LINE_STATE
            Serial.println("3 checking for line");
        #endif
        if(imOnLine) {
            #if DEBUG_LINE_STATE
                Serial.println("3 to 2, enter sent");
            #endif
            lineState = 2;
        }
    }
}

bool LSystem::case2Check(float prevDir, float rot) {
    prevDir = floatMod(prevDir - rot, 360);
    if (prevDir > 180) {
        prevDir = prevDir - 360;
    }
    for (int i = 0; i < NUM_LS; i++) {
        if (sensorIsWhite[i]) {
            float sensorAngle = i * (360.0f / NUM_LS);
            if (sensorAngle > 180) {
                sensorAngle = sensorAngle - 360;
            }
            float currvsprevdiff = circularDiff(sensorAngle, prevDir);
            bool condition = (clusterAmount <= 2) ? (currvsprevdiff > 90 && currvsprevdiff < 270) : (currvsprevdiff > 145 && currvsprevdiff < 215);
            if(condition) {
                return true;
            }
        }
    }
    return false; // Return false if no white sensor is found
}
