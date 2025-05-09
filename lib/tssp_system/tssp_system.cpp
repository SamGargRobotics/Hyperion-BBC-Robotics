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
        readTssp[i] = 0;
        tsspSortedValuesNormal[i] = 0;
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
            if(readTssp[i] > tsspSortedValuesNormal[j]) {
                if(j <= i) {
                    ARRAYSHIFTDOWN(tsspSortedValuesNormal, j, i);
                }
                tsspSortedValuesNormal[j] = readTssp[i];
                break;
            }
        }
    }

    for(uint8_t i = 0; i < TSSPNUM, i++) {
        if(readTssp[i] == 0) {
            detectingBall = false;
        } else {
            detectingBall = true;
            break;
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

    for(uint8_t i = 0; i < TSSPNUM; i++){
        readTssp[i] = (readTssp[i] == 255 || readTssp[i] == -1) ? (readTssp[i+1] + readTssp[i-1])/2 : readTssp[i];
    }

    normalBallDir = (highestTssp)*(360.0/TSSPNUM);
    ballStr = ((3 * tsspSortedValuesNormal[0]) + (2 * tsspSortedValuesNormal[1]) + tsspSortedValuesNormal[2] + tsspSortedValuesNormal[3]) / 7;
}