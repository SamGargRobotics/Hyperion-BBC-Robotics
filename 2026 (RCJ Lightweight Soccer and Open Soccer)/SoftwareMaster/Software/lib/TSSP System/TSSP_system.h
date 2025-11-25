#ifndef TSSP_SYSTEM_H
#define TSSP_SYSTEM_H

#include <Arduino.h>
#include <Configuration.h>
#include <Common.h>
#include <Pins.h>

class BallInfo {
public:
    float str() const { return _str; }
    float dir() const { return _dir; }
private:
    friend class TsspSystem;
    float _dir = 0;
    float _str = 0;
};

class MoveInfo {
public:
    float spd() const { return _spd; }
    float dir() const { return _dir; }
private:
    friend class TsspSystem;
    float _dir = 0;
    float _spd = 0;
};

class TsspSystem {
public:
    void init();
    void update();
    const BallInfo&   ball()  const { return bInfo; }
    const MoveInfo&   move()  const { return mInfo; }
private:
    #define TSSP_NUM 16
    uint8_t tsspPins[TSSP_NUM] = {TSSP1, TSSP2, TSSP3, TSSP4, TSSP5, TSSP6, 
                                 TSSP7, TSSP8, TSSP9, TSSP10, TSSP11, TSSP12,
                                 TSSP13, TSSP14, TSSP15, TSSP16};
    MoveInfo mInfo;
    BallInfo bInfo;
};

#endif
