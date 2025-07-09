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

    float getBallStr();
    float getBallDir();
private:
    //! @brief How far away the ball in units
    float ballStr = 0;
    //! @brief Direction of the ball
    float ballDir = 0;
    uint8_t tsspPins[TSSPNUM] = {TSSP1, TSSP2, TSSP3, TSSP4, TSSP5, TSSP6, 
                                 TSSP7, TSSP8, TSSP9, TSSP10, TSSP11, TSSP12,
                                 TSSP13, TSSP14, TSSP15, TSSP16};
};

#endif