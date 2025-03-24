/*!
 * @file tssp_system.cpp
 * 
 * @mainpage Multiple TSSP58038 Sensor's
 * 
 * This is a library for multiple TSSP50838 Sensor
*/
#include "tssp_system.h"
#include <configandpins.h>

void Tssp_system::init() {
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        pinMode(tsspPins[i], INPUT);
    }
}

/*!
 * @brief Reads the TSSP50838's (multiple at once)
*/
void Tssp_system::read() {
    // Read the tssp itself
    for(int y = 0; y < 255; y++) {
        for(uint8_t i = 0; i < TSSPNUM; i++) {
            readTssp[i] += (1 - digitalRead(tsspPins[i]));
        }
        delayMicroseconds(3);
    }
    
    // Check for any broken sensors (so that we can ignore them in our code later)
    for (uint8_t i = 0; i < TSSPNUM; i++) {
        readingTsspIgnores[i] = (readTssp[i] == 255 || readTssp[i] == -1) ? 1 : 0;
    }

    // Find the sensor with the highest reading (meaning the ball is nearest to that sensor)
    for (uint8_t i = 0; i < TSSPNUM; i++) {
        if(readingTsspIgnores[i] == 0) {
            if(readTssp[i] > largestReading) {
                largestReading = readTssp[i];
                highestTssp = i;
            }
        }
    }

    for (uint8_t i = 0; i < TSSPNUM; i++) {
        if(readingTsspIgnores[i] == 0) {
            if(readTssp == 0) {
                detectingBall = false;
            } else {
                detectingBall = true;
                break;
            }
        }
    }

    // Deviation reading (See function for explanation) - Is only used when deviationFunctionToggle is activated.
    if(deviationFunctionToggle) {
        ballDirOffset = deviationReading(highestTssp); // [NOTE FOR EDITOR] ADD THIS TO FINAL BALL DIR VALUE
    }

    // Add up all the readings from the tssps to calculate ballStr reading later
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        if(readingTsspIgnores[i] == 0) {
            addedAngles += readTssp[i];
        }
    }

    //Final Calculations
    ballStr = addedAngles/TSSPNUM;
    ballDir = (highestTssp-1)*30;
}

/*!
 * @brief Approximates the exact angle of the ball based on readings from the
 *        tssp higher and lower to the one reading the highest value.
 * 
 * @param midTssp Tssp that reads the highest value.
 * 
 * @return Offset value of the current direction that the main function is
 *         reading.
*/
int Tssp_system::deviationReading(int midTssp) {
    // Take all necessary readings relative to the tssp with the highest reading (one above the tssp and one below the tssp)
    // First check that midTssp isn't of two other special cases, if not, assume that midTssp is of a normal case.
    if(midTssp == TSSPNUM) {
        midTsspReading = readTssp[midTssp];
        followingTsspReading = readTssp[midTssp-1];
        leadingTsspReading = readTssp[0];
    } else if(midTssp == 0) {
        midTsspReading = readTssp[midTssp];
        followingTsspReading = readTssp[TSSPNUM];
        leadingTsspReading = readTssp[midTssp+1];
    } else {
        midTsspReading = readTssp[midTssp];
        followingTsspReading = readTssp[midTssp-1];
        leadingTsspReading = readTssp[midTssp+1];
    }

    // Finding if either the one above, or one below has the bigger reading. By doing this, we know which index we can get to calculate the offset.
    leadingTsspBigger = leadingTsspReading >= followingTsspReading ? 1 : 0;

    // Subtract the readings depending on which value is bigger, by doing this, an offset value is created between the highest tssp value, and the highest value of the two tssps beside the highest valued tssp.
    if(leadingTsspBigger) {
        deviationOffset = midTsspReading - leadingTsspReading;
    } else {
        deviationOffset = midTsspReading - followingTsspReading;
    }

    // Multiply the deviation offset by a constant, to turn it into a more accurate offset reading.
    return deviationOffset * TSSP_DEVIATION_CONSTANT;
}