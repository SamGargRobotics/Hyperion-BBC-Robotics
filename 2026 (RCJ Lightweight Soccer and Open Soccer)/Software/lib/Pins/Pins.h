#ifndef PINS_H
#define PINS_H

#include <Arduino.h>
#include <Configuration.h>

#if ROBOT
    #define FRINA 0
    #define FRINB 0
    #define FRPWM 0
    #define BRINA 0
    #define BRINB 0
    #define BRPWM 0
    #define BLINA 0
    #define BLINB 0
    #define BLPWM 0
    #define FLINA 0
    #define FLINB 0
    #define FLPWM 0
#else
    #define FRINA 0
    #define FRINB 0
    #define FRPWM 0
    #define BRINA 0
    #define BRINB 0
    #define BRPWM 0
    #define BLINA 0
    #define BLINB 0
    #define BLPWM 0
    #define FLINA 0
    #define FLINB 0
    #define FLPWM 0
#endif
#endif