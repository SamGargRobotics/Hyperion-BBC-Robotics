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
 * @brief Reading the tssps and calculating any that are broken to ignor later
 */
void Tssp_system::read() {
    for(int i = 0; i < TSSPNUM; i++) {
        readTssp[i] = 0;
    }

    largestReading = 0;
    highestTssp = 0;
    detectingBall = false;

    for(int y = 0; y < 255; y++) {
        for(uint8_t i = 0; i < TSSPNUM; i++) {
            readTssp[i] += (1 - digitalRead(tsspPins[i]));
        }
        delayMicroseconds(30);
    }

    #if TSSP_DEBUG_SPEC_SENSOR
        for(int y = 0; y < 255; y++) {
            for(uint8_t i = 0; i < TSSPNUM; i++) {
                oneSensorReading += (1-digitalRead(TSSP1));
            }
        }
        Serial.println(oneSensorReading);
    #endif

    for (uint8_t i = 0; i < TSSPNUM; i++) {
        readingTsspIgnores[i] = (readTssp[i] == 255 || readTssp[i] == -1) ?
                                true : false;
    }

    for (int i = 0; i < TSSPNUM; i++) {
        if (readingTsspIgnores[i] == 0) {  // Only check non-ignored sensors
            if (readTssp[i] != 0 && readTssp[i] != 255) {  // Ball is seen at this index
                detectingBall = true;
                break;  // Exit the loop since ball is found
            }
        }
    }
}

/*!
 * @brief Reads the TSSP50838's in a manner that has little to no maths.
 */
void Tssp_system::normalCalc() {
    for(int i = 0; i < TSSPNUM; i++) {
        readingTsspIgnores[i] = false;
    }

    normalBallDir = 0;
    addedAngles = 0;
    oneSensorReading = 0;

    read();

    // Find the sensor with the highest reading (meaning the ball is nearest to 
    // that sensor)
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

    #if DEBUG_TSSP_SENSOR_VAL
        for(uint8_t i = 0; i < TSSPNUM; i++) {
            Serial.print(readTssp[i]);
            Serial.print("\t");
        }
        Serial.println();
    #endif

    // Add up all the readings from the tssps to calculate ballStr reading later
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        if(readingTsspIgnores[i] == 0) {
            addedAngles += readTssp[i];
        }
    }

    //Final Calculations
    ballStr = addedAngles/TSSPNUM;
    normalBallDir = ((highestTssp == 0)?highestTssp:(highestTssp-1))*(360/TSSPNUM);
}

/*!
 * @brief Reads the TSSP50838's in a manner that has alot of maths.
 */
void Tssp_system::advancedCalc() {
    advancedBallDir = 0;

    read();

    // Find the top 4 highest values
    for(int i = 0; i < TSSPNUM; i++) {
        if(readTssp[i] > top4Tssps[0]) {
            top4Tssps[0] = i;
        }
    }
    for(int i = 0; i < TSSPNUM; i++) {
        if(i != top4Tssps[0]) {
            if(readTssp[i] > top4Tssps[1]) {
                top4Tssps[1] = i;
            }
        }
    }
    for(int i = 0; i < TSSPNUM; i++) {
        if(i != top4Tssps[0] && i != top4Tssps[1]) {
            if(readTssp[i] > top4Tssps[2]) {
                top4Tssps[2] = i;
            }
        }
    }
    for(int i = 0; i < TSSPNUM; i++) {
        if(i != top4Tssps[0] && i != top4Tssps[1] && i != top4Tssps[2]) {
            top4Tssps[3] = i;
        }
    }

    //Calculate X and Y components of angle using unit circle
    xcomp = (top4Tssps[0] * tsspXComponents[top4Tssps[0]]) + \
            (top4Tssps[1] * tsspXComponents[top4Tssps[1]]) + \
            (top4Tssps[2] * tsspXComponents[top4Tssps[2]]) + \
            (top4Tssps[3] * tsspXComponents[top4Tssps[3]]);

    ycomp = (top4Tssps[0] * tsspYComponents[top4Tssps[0]]) + \
            (top4Tssps[1] * tsspYComponents[top4Tssps[1]]) + \
            (top4Tssps[2] * tsspYComponents[top4Tssps[2]]) + \
            (top4Tssps[3] * tsspYComponents[top4Tssps[3]]);

    //Create arctan function to calculate angle (like physics as sam understands it 💀)
    advancedBallDir = atanf(xcomp/ycomp) * RAD_TO_DEG;

    //Ensure that the special cases in the unit circle and basic angle maths is accounted for
    if(advancedBallDir < 0) {
        advancedBallDir += 180;
    }

    if(xcomp < 0) {
        advancedBallDir += 180;
    }
    
    // Calculate Ball Strength
    advancedBallStr = ((top4Tssps[0] * FIRST_HIGHEST_TSSP_STR_MULTIPLIER) + \
                      (top4Tssps[1] * SECOND_HIGHEST_TSSP_STR_MULTIPLIER) + \
                      (top4Tssps[2] * THIRD_HIGHEST_TSSP_STR_MULTIPLIER) + \
                      (top4Tssps[3] * FOURTH_HIGHEST_TSSP_STR_MULTIPLIER)) / \
                      (FIRST_HIGHEST_TSSP_STR_MULTIPLIER + 
                      SECOND_HIGHEST_TSSP_STR_MULTIPLIER +
                      THIRD_HIGHEST_TSSP_STR_MULTIPLIER  + 
                      FOURTH_HIGHEST_TSSP_STR_MULTIPLIER );
}