#ifndef TSSP_SYSTEM_H
#define TSSP_SYSTEM_H

#include <Arduino.h>
#include <Configuration.h>
#include <Common.h>
#include <Pins.h>

/*!
* @brief tssp Class that contains all functions for IR ⚽👀
*/

class TsspSystem {
public:
    void init();
    void read_all_tssp();
    float get_ball_dir();
    int get_ball_str();
private:
    #define TSSP_NUM 16
    uint8_t tsspPins[TSSP_NUM] = {TSSP1, TSSP2, TSSP3, TSSP4, TSSP5, TSSP6, 
                                 TSSP7, TSSP8, TSSP9, TSSP10, TSSP11, TSSP12,
                                 TSSP13, TSSP14, TSSP15, TSSP16};
    float ballDir = 0;
    int ballStr = 0;
};

#endif
