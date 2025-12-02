#ifndef TSSP_SYSTEM_H
#define TSSP_SYSTEM_H

#include <Arduino.h>
#include <Common.h>

class TsspSystem {
public:
    void init();
    void update();

    float get_ball_dir() const { return ballDir; }
    float get_ball_str() const { return ballStr; }
private:
    #define TSSP_NUM 16
    uint8_t tsspPins[TSSP_NUM] = {0};
    float ballDir = 0;
    float ballStr = 0;
    float tsspX[TSSPNUM] = {0};
    float tsspY[TSSPNUM] = {0};
};

#endif