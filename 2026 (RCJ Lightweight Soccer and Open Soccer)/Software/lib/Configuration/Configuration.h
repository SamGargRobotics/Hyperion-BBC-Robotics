#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

#define ROBOT 1
#define GOAL_TRACKING_TOGGLE 0

#if ROBOT
    #define ORBIT_TUNER 1
    #define BASE_SPEED 0
    #define SURGE_SPEED 0
    #define KP_IMU 0
    #define KD_IMU 0
    #define KP_GOALT 0
    #define KD_GOALT 0
    #define KP_VERT 0
    #define KD_VERT 0
    #define KP_HOZT 0
    #define KD_HOZT 0
    #define SP_DEFEND_VERT 0
    #define DEF_START_SURGE 0
    #define DEF_KEEP_SURGE_UNTIL 0
#else
    #define ORBIT_TUNER 1
    #define BASE_SPEED 0
    #define SURGE_SPEED 0
    #define KP_IMU 0
    #define KD_IMU 0
    #define KP_GOALT 0
    #define KD_GOALT 0
    #define KP_VERT 0
    #define KD_VERT 0
    #define KP_HOZT 0
    #define KD_HOZT 0
    #define SP_DEFEND_VERT 0
    #define DEF_START_SURGE 0
    #define DEF_KEEP_SURGE_UNTIL 0
#endif

#define BT_SERIAL Serial1
#define BT_BAUD 9600
#define BT_PACKET_SIZE 6
#define BT_START_BYTE 255
#define BT_NO_DATA 255
#define BT_PERFORMANCE_PERCENT 10

#define cameraSerial Serial8
#define CAM_PACKET_SIZE 8
#define CAM_START_PACK_1 200
#define CAM_START_PACK_2 122

#define KICKER_VOLTAGE_STABALISER 1
#define KICKER_REQUIRED_VOLT 0
#define ROBOT_VOLTAGE_STABALISER 1 
#define ROBOT_REQUIRED_VOLT 0

#endif