#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>

class Common {
public:
    float angleBetween(float angleCounterClockwise, float angleClockwise);
    float midAngleBetween(float angleCounterClockwise, float angleClockwise);
    int intMod(int x, int m);
private:
};

#endif