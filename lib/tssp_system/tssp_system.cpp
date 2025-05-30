/*!
 * @file tssp_system.cpp
 * 
 * @mainpage Multiple TSSP58038 Sensor's
 * 
 * This is a library for multiple TSSP50838 Sensor
 */
#include "tssp_system.h"

/*!
 * @brief Initialize all the tssp's accordingly
 */
void Tssp_system::init() {
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        pinMode(tsspPins[i], INPUT);
    }
}

/*! 
 * @brief Completes calculations for ballDir and ballStr by reading the Tssps
 */
void Tssp_system::update() {
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        tsspX[i] = sin(i*22.5*DEG_TO_RAD); 
        tsspY[i] = cos(i*22.5*DEG_TO_RAD);
    }
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        readTssp[i] = 0;
        tsspSortedValues[i] = 0;
    }
    largestReading = 0;
    highestTssp = 0;
    detectingBall = false;

    for(uint8_t i = 0; i < 255; i++) {
        for(uint8_t j = 0; j < TSSPNUM; j++) {
            readTssp[j] += 1 - digitalRead(tsspPins[j]);
        }
        delayMicroseconds(10);
    }

    #if DEBUG_TSSP_SENSOR_VAL 
        for(uint8_t i = 0; i < TSSPNUM; i++) {
            Serial.print(readTssp[i]);
            Serial.print("\t");
        }
        Serial.println();
    #endif

    for(uint8_t i = 0; i < TSSPNUM; i++) {
        for(uint8_t j = 0; j < TSSPNUM; j++) {
            if(readTssp[i] > tsspSortedValues[j]) {
                if(j <= i) {
                    ARRAYSHIFTDOWN(tsspSortedValues, j, i);
                    ARRAYSHIFTDOWN(tsspSortedIndex, j, i);
                }
                tsspSortedValues[j] = readTssp[i];
                tsspSortedIndex[j] = i;
                break;
            }
        }
    }

    for (uint8_t i = 0; i < TSSPNUM; i++) {
        if(readingTsspIgnores[i] == 0) {
            if(readTssp[i] > largestReading) {
                largestReading = readTssp[i];
                highestTssp = i;
            }
        }
    }

    for(uint8_t i = 0; i < TSSPNUM; i++) {
        readTssp[i] = (readTssp[i] == 255 || readTssp[i] == -1) ? (readTssp[i+1] + readTssp[i-1])/2 : readTssp[i];
    }

    ballDir = (highestTssp)*(360.0/TSSPNUM);
    ballStr = ((3 * tsspSortedValues[0]) + (2 * tsspSortedValues[1]) + tsspSortedValues[2] + tsspSortedValues[3]) / 7;
    detectingBall = (ballStr != 0);

    float x = ((tsspSortedValues[0] * tsspX[tsspSortedIndex[0]]) + (tsspSortedValues[1] * tsspX[tsspSortedIndex[1]]) + (tsspSortedValues[2] * tsspX[tsspSortedIndex[2]]) + (tsspSortedValues[3] * tsspX[tsspSortedIndex[3]])) / 4;
    float y = ((tsspSortedValues[0] * tsspY[tsspSortedIndex[0]]) + (tsspSortedValues[1] * tsspY[tsspSortedIndex[1]]) + (tsspSortedValues[2] * tsspY[tsspSortedIndex[2]]) + (tsspSortedValues[3] * tsspY[tsspSortedIndex[3]])) / 4;
    ballDir = detectingBall ? 360 - floatMod((RAD_TO_DEG * (atan2f(y, x)))-90, 360) : 0;
    Serial.println(ballDir);

    for(int i = 10; i > 0; i--) {
        previousBallDir[i] = previousBallDir[i-1];
        previousBallStr[i] = previousBallStr[i-1];
    }
}