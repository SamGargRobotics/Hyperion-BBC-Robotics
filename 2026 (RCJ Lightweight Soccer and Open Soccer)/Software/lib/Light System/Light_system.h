#ifndef LIGHT_SYSTEM_H
#define LIGHT_SYSTEM_H

#include <Arduino.h>
#include <Pins.h>
#include <Configuration.h>
#include <Common.h>
#include <EEPROM.h>

class LightSystem {
public:
    void init();
    void calibrate();
    float get_line_dir();
    void inner_circle_direction_calc(float rot, bool motorOn);
private:
    #define NUM_LS 32
    int pinList[4] = {LIGHT_PIN_DIGI_0, LIGHT_PIN_DIGI_1, LIGHT_PIN_DIGI_2,
                        LIGHT_PIN_DIGI_3};
    int whiteThreshold[NUM_LS] = {0};
    bool sensorIsWhite[NUM_LS] = {0};
    int read_one(int sensorNum);
    void calculate_line_state(float rot, float lineDirection, float linePos);
    float calculate_distance_over(float angle1, float angle2);
    float lineState = 0;
    float lineDir = -1;
    int maxIndex = 44;
    int minIndex = 44;
    int clustersList[4][2];
    float clusterCenter[3] = {44, 44, 44};

    Common com;
};

#endif