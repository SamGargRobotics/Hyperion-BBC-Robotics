#ifndef LIGHT_SYSTEM_H
#define LIGHT_SYSTEM_H

#include <Arduino.h>
#include <Pins.h>
#include <Configuration.h>
#include <Common.h>
#include <EEPROM.h>

class LightSystem {
public:
    
private:
    #define NUM_LS 32
    #define OUTER_NUM_LS 16
    #define LS_FLIP_THRESH 90

    Common com;
};

#endif