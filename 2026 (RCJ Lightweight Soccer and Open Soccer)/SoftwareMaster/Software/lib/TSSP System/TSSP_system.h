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
    float _mag = 0;
};

class TsspSystem {
public:
    void init();
    void update();
    const BallInfo&   ball()  const { return bInfo; }
    const MoveInfo&   move()  const { return mInfo; }

private:
    void readUartData();   // new function for receiving UART packets

    BallInfo bInfo;
    MoveInfo mInfo;

    String uartBuffer = "";   // stores incoming serial line temporarily
};

#endif
