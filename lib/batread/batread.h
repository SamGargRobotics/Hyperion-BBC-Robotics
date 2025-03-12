#ifndef BATREAD_H
#define BATREAD_H

#include <Arduino.h>
#include <configandpins.h>

class BatRead { 
public:
    BatRead() {};
    void init();
    float read();
private:
    float rawValue = 0;
    float highest = 2.9302325581;
    float lowest = 2.651167907;
};

#endif