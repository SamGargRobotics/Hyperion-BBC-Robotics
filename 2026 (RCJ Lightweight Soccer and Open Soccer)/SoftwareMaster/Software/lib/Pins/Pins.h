#ifndef PINS_H
#define PINS_H

#include <Arduino.h>
#include <Configuration.h>

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
#define DRINA 0
#define DRINB 0
#define DRPWM 0

#define LIGHT_PIN_DIGI_0 0
#define LIGHT_PIN_DIGI_1 0
#define LIGHT_PIN_DIGI_2 0
#define LIGHT_PIN_DIGI_3 0
#define LIGHT_PIN_DIGI_4 0
#define LIGHT_PIN_DIGI_5 0

#define KICKER_PIN 0

#define KICKER_VD_PIN 0 
#define ROBOT_VD_PIN 0

/*
    SWITCHES:
    - Power (Hardware Switch) - Powers motors and comm module.
    - Logic (Hardware Switch) - Powers all logic based systems.
    - Calibration (Software Switch) - Allows for calibration of IMU and LS.
    - Motor (Software Switch) - No motors are run unless this switch is turned on.
    - Dribbler (Software Switch) - Dribbler is not to run unless this switch is turned on.
*/
#define CALIBRATION_SWITCH 0
#define MOTOR_SWITCH 0

#endif