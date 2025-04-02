/*!
 * @file config.h
 * 
 * This is a library that includes variables and defines that change the code's
 * behaviour.
 * 
 * S.Garg (Brisbane Boys' College)
 * T.McCabe (Brisbane Boys' College)
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

//! @def MOTORNUM @brief Number of motors.
#define MOTORNUM 4

//! @def TSSPNUM @brief Number of Tssps on the robot
#define TSSPNUM 16

//! @def NUM_LS @brief Number of light sensors
#define NUM_LS 32
//! @def NUMBER_MUX @brief Number of mux's
#define NUMBER_MUX 2

//! @def PID_p @brief Proportional aspect of PID
#define PID_p 1
//! @def PID_i @brief Intergral aspect of PID
#define PID_i 1
//! @def PID_d @brief Derivative aspect of PID
#define PID_d 1
//! @def PID_abs_max @brief Absoloute max of PID
#define PID_abs_max 1

//! @def EXPO_MIN_VAL @brief Minimum value of the exponential orbit
#define EXPO_MIN_VAL 60
//! @def ORBIT_MULTIPLIER @brief Multiplier for the exponential orbit
#define ORBIT_MULTIPLIER 2.71828182846

//! @def GOAL_SEMI_CIRCLE_RADIUS_CM @brief The defender's arc orbit around goal
#define GOAL_SEMI_CIRCLE_RADIUS_CM 10

#endif