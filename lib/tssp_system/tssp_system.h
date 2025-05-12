/*!
 * @file tssp_system.h
 * 
 * This is a library for the TSSP58038 to read multiple TSSP58038's at once.
 * 
 * S.Garg (Brisbane Boys' College)
 */

#ifndef TSSP_SYSTEM_H
#define TSSP_SYSTEM_H

#include <config.h>
#include <common.h>
#include <pins.h>
#include <common.h>

/*!
 * @brief Class that stores state and functions for interacting with multiple 
          TSSP58038's at once.
 */
class Tssp_system {
public:
    Tssp_system() {};
    void init();
    void update();
    bool detectingBall = true;
    float ballStr = 0;
    float ballDir = 0;
    float advancedBallDir = 0;
    int highestTssp = 0;
private:
    bool deviationFunctionToggle = false;
    uint8_t readTssp[TSSPNUM] = {0};
    uint8_t tsspSortedValues[TSSPNUM] = {0};
    uint8_t tsspSortedIndex[TSSPNUM] = {0};
    uint8_t tsspPins[TSSPNUM] = {TSSP1, TSSP2, TSSP3, TSSP4, TSSP5, TSSP6, TSSP7,
                            TSSP8, TSSP9, TSSP10, TSSP11, TSSP12, TSSP13, 
                            TSSP14, TSSP15, TSSP16};
    int readingTsspIgnores[TSSPNUM] = {0};
    int largestReading = 0;
    float xcomp = 0;
    float ycomp = 0;
    float tsspXComponents[TSSPNUM] = {TSSP1_X, TSSP2_X, TSSP3_X, TSSP4_X, 
                                    TSSP5_X, TSSP6_X, TSSP7_X, TSSP8_X, TSSP9_X,
                                    TSSP10_X, TSSP11_X, TSSP12_X, TSSP13_X, 
                                    TSSP14_X, TSSP15_X, TSSP16_X};
    float tsspYComponents[TSSPNUM] = {TSSP1_Y, TSSP2_Y, TSSP3_Y, TSSP4_Y, TSSP5_Y, 
                                    TSSP6_Y, TSSP7_Y, TSSP8_Y, TSSP9_Y, 
                                    TSSP10_Y, TSSP11_Y, TSSP12_Y, TSSP13_Y, 
                                    TSSP14_Y, TSSP15_Y, TSSP16_Y};
};

#endif