#ifndef LIGHT_SYSTEM_H
#define LIGHT_SYSTEM_H

#include <Arduino.h>

Class LightSystem {
public:
    void init();
private:
    int pinList[4] = {LIGHT_PIN_DIGI_0, LIGHT_PIN_DIGI_1, LIGHT_PIN_DIGI_2,
                        LIGHT_PIN_DIGI_3};
    int whiteThreshold[NUM_LS] = {0};
    int sensorIsWhite[NUM_LS] = {0};
    int read_one(int sensorNum);
};

#endif