/*!
 * @file configandpins.h
 * 
 * This is a library used widely across most libraries as it contains
 * customizable information all in one file.
 * 
 * S.Garg (Brisbane Boys' College)
 * T.McCabe (Brisbane Boys' College)
 */
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
#define FRPWM 25
#define FRINA 27
#define FRINB 26
#define FLPWM 11
#define FLINA 24
#define FLINB 12
#define BRPWM 28
#define BRINA 30
#define BRINB 29
#define BLPWM 8
#define BLINA 10
#define BLINB 9

#define BALL_DIS_MULTIPLIER 1
#define TSSPNUM 16
#define TSSP_DEVIATION_CONSTANT 0
#define BALL_STRENGTH_MULTIPLIER 1
#define TSSP1 16
#define TSSP2 13
#define TSSP3 41
#define TSSP4 40
#define TSSP5 39
#define TSSP6 37
#define TSSP7 36
#define TSSP8 33
#define TSSP9 32
#define TSSP10 7
#define TSSP11 6
#define TSSP12 5
#define TSSP13 4
#define TSSP14 3
#define TSSP15 2
#define TSSP16 17

#define NUM_LS 32
#define NUMBER_MUX 2
#define LS_OFFSET 1
#define LIGHT_PIN 21
#define LIGHT_PIN2 20
#define LIGHT_PIN_DIGI_0 23
#define LIGHT_PIN_DIGI_1 22
#define LIGHT_PIN_DIGI_2 15
#define LIGHT_PIN_DIGI_3 14

#define BAT_READ_PIN 38

#define PID_p 1
#define PID_i 1
#define PID_d 1
#define PID_abs_max 1

#define EXPO_MIN_VAL 60
#define ORBIT_MULTIPLIER 2.71828182846

#define GOAL_SEMI_CIRCLE_RADIUS_CM 10

#define BLUETOOTH_SERIAL Serial1
#define BLUETOOTH_BAUD 9600
#define BLUETOOTH_PACKET_SIZE 5
#define BLUETOOTH_START_BYTE 254
#define BLUETOOTH_NO_DATA 255   

#endif