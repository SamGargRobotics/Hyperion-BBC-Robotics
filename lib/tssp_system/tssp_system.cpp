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
    // Assign tssp x and y components based on place in unit circle (vs real)
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        tsspX[i] = sin(i*(360/TSSPNUM)*DEG_TO_RAD); 
        tsspY[i] = cos(i*(360/TSSPNUM)*DEG_TO_RAD);
    }
    // Zero all arrays from previous loop
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        readTssp[i] = 0;
        tsspSortedValues[i] = 0;
    }

    // Read the tssps themselves.
    for(uint8_t i = 0; i < 255; i++) {
        for(uint8_t j = 0; j < TSSPNUM; j++) {
            readTssp[j] += 1 - digitalRead(tsspPins[j]);
        }
        delayMicroseconds(10);
    }

    // Prints each individual sensor value out if true.
    #if DEBUG_TSSP_SENSOR_VAL 
        for(uint8_t i = 0; i < TSSPNUM; i++) {
            Serial.print(readTssp[i]);
            Serial.print("\t");
        }
        Serial.println();
    #endif
    
    // If any sensors are broken, the average of the 2 sensors beside it become 
    // it's value
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        readTssp[i] = (readTssp[i] == 255 || readTssp[i] == -1) ? \
                                (readTssp[i+1] + readTssp[i-1])/2 : readTssp[i];
    }

    // tsspSortedValues: Sorts all readTssp values in descending order
    // tsspSortedIndex: Sorts index's of readTssp in descending
    //                  order corresponding to the value it reads.
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

    // Average the top 4 readings from each of the sensors by their
    // corresponding x and y components
    float x = ((tsspSortedValues[0] * tsspX[tsspSortedIndex[0]]) + \
              (tsspSortedValues[1] * tsspX[tsspSortedIndex[1]]) + \
              (tsspSortedValues[2] * tsspX[tsspSortedIndex[2]]) + \
              (tsspSortedValues[3] * tsspX[tsspSortedIndex[3]])) / 4;
    float y = ((tsspSortedValues[0] * tsspY[tsspSortedIndex[0]]) + \
              (tsspSortedValues[1] * tsspY[tsspSortedIndex[1]]) + \
              (tsspSortedValues[2] * tsspY[tsspSortedIndex[2]]) + \
              (tsspSortedValues[3] * tsspY[tsspSortedIndex[3]])) / 4;
    
    // ballStr: Weighted average of the top 4 sensors.
    // detectingBall: If ball strength is 0 (all sensors read 0), then ball isnt 
    //                there.
    // ballDir: Using vector calculations, a certain radian value can be found 
    //          and then that is converted to degrees.
    ballStr = ((3 * tsspSortedValues[0]) + (2 * tsspSortedValues[1]) + \
              tsspSortedValues[2] + tsspSortedValues[3]) / 7;
    detectingBall = (ballStr != 0);
    ballDir = detectingBall ? 360 - \
                            floatMod((RAD_TO_DEG * (atan2f(y, x)))-90, 360) : 0;
}