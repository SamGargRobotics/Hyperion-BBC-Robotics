#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define ROBOT 1

#if ROBOT
    #define ORBIT_TUNER 1
    #define BASE_SPEED 0
    #define SURGE_SPEED 0
    #define KP_IMU 0
    #define KD_IMU 0
#else
    #define ORBIT_TUNER 1
    #define BASE_SPEED 0
    #define SURGE_SPEED 0
    #define KP_IMU 0
    #define KD_IMU 0
#endif

#define BT_SERIAL Serial1
#define BT_BAUD 9600
#define BT_PACKET_SIZE 6
#define BT_START_BYTE 255
#define BT_NO_DATA 255
#define BT_PERFORMANCE_PERCENT 10

#define KICKER_VOLTAGE_STABALISER 1
#define KICKER_REQUIRED_VOLT 0

#endif