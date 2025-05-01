/*!
 * @file config.h
 * 
 * @mainpage This is a library that includes variables and defines that change 
 * the code's behaviour.
 * 
 * @date 30/04/25
 * 
 * @author S.Garg (Brisbane Boys' College)
 * @author T.McCabe (Brisbane Boys' College)
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
//! @def attackingGoal @brief Blue = 1, Yelow = 0; Assigns which goal attacking
#define attackingGoal true
//! @def CORRECTION_TEST @brief Testing Correction Only if True
#define CORRECTION_TEST false
//! @def GOAL_TRACKING_TOGGLE @brief If the robot should goal track
#define GOAL_TRACKING_TOGGLE 1
//! @def BALL_FOLLOW_TEST @brief Testing Corretion + Ball Follow if true
#define BALL_FOLLOW_TEST false
//! @def SET_SPEED @brief Speed that is set for running
#define SET_SPEED 75

//! @def MOTORNUM @brief Number of motors.
#define MOTORNUM 4

//! @def TSSPNUM @brief Number of Tssps on the robot
#define TSSPNUM 16

//! @def NUM_LS @brief Number of light sensors
#define NUM_LS 32
//! @def NUMBER_MUX @brief Number of mux's
#define NUMBER_MUX 2

//! @def PID_p @brief Proportional aspect of PID
#define PID_p 0.9 //0.9
//! @def PID_i @brief Intergral aspect of PID
#define PID_i 0
//! @def PID_d @brief Derivative aspect of PID
#define PID_d 0.075 //0.075
//! @def PID_abs_max @brief Absoloute max of PID
#define PID_abs_max 100

//! @def BAT_MOTOROFF_THRESH @brief Thresh to determine if motor switch off
#define BAT_MOTOROFF_THRESH 0.5

//! @def EXPO_MIN_VAL @brief Minimum value of the exponential orbit
#define EXPO_MIN_VAL 75
//! @def ORBIT_MULTIPLIER @brief Multiplier for the exponential orbit
#define ORBIT_MULTIPLIER 2.71828182846

//! @def GOAL_SEMI_CIRCLE_RADIUS_CM @brief The defender's arc orbit around goal
#define GOAL_SEMI_CIRCLE_RADIUS_CM 10
//! @def ORBIT_STRENGTH_RADIUS @brief The strength value that the robot switches
// strats for orbit
#define ORBIT_STRENGTH_RADIUS 45

//! @def BATTERY_CRITICAL @brief The battery level where the battery is critical
#define BATTERY_CRITICAL 11.7

//! @def cameraSerial @brief Serial that the camera sends data over to
#define cameraSerial Serial8

#define DEBUG (DEBUG_MOTORS || DEBUG_IMU || DEBUG_TSSP_SENSOR_VAL || DEBUG_TSSP_IGNORE || TSSP_DEBUG_SPEC_SENSOR || DEBUG_TSSP);
#define DEBUG_MOTORS false
#define DEBUG_IMU true
#define DEBUG_TSSP false
#define DEBUG_TSSP_SENSOR_VAL false
#define DEBUG_TSSP_IGNORE false
#define TSSP_DEBUG_SPEC_SENSOR false
#define BAT_READ_RAWVAL false
#define BAT_READ_VOLTS false
#define DEBUG_GOAL_TRACKING false

#endif