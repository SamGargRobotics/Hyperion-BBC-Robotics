#ifndef CONFIGANDPINS_H
#define CONFIGANDPINS_H

#include <Arduino.h>

float floatMod(float x, float m);
 
float angleBetween(float angleCounterClockwise, float angleClockwise);
 
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise);
 
int8_t findSign(float value);
 
float midAngleBetween(float angleCounterClockwise, float angleClockwise);
 
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise);
 
int8_t findSign(float value);
 
bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise, float angleCheck);

#define MOTORNUM 4
#define FRPWM 0
#define FRINA 0
#define FRINB 0
#define FLPWM 0
#define FLINA 0
#define FLINB 0
#define BRPWM 0
#define BRINA 0
#define BRINB 0
#define BLPWM 0
#define BLINA 0
#define BLINB 0

#define BALL_DIS_MULTIPLIER 1
#define TSSPNUM 16
#define TSSP_DEVIATION_CONSTANT 0
#define BALL_STRENGTH_MULTIPLIER 1
#define TSSP1 0
#define TSSP2 0
#define TSSP3 0
#define TSSP4 0
#define TSSP5 0
#define TSSP6 0
#define TSSP6 0
#define TSSP7 0
#define TSSP8 0
#define TSSP9 0
#define TSSP10 0
#define TSSP11 0
#define TSSP12 0
#define TSSP13 0
#define TSSP14 0
#define TSSP15 0
#define TSSP16 0

#define NUM_LS 1
#define NUMBER_MUX 1
#define LS_OFFSET 1
#define LIGHT_PIN 1
#define LIGHT_PIN_DIGI_0 1
#define LIGHT_PIN_DIGI_1 1
#define LIGHT_PIN_DIGI_2 1
#define LIGHT_PIN_DIGI_3 1

#define PID_p 1
#define PID_i 1
#define PID_d 1
#define PID_abs_max 1

#define EXPO_MIN_VAL 60
#define ORBIT_MULTIPLIER 2.71828182846

#define BAT_READ_PIN 38

#define GOAL_SEMI_CIRCLE_RADIUS_CM 10

#endif