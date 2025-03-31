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

//! @def MOTORNUM @brief Number of motors.
#define MOTORNUM 4
//! @def FRPWM @brief Front Right Motor Power Pin
#define FRPWM 25
//! @def FRINA @brief Front Right Motor Input A
#define FRINA 27
//! @def FRINB @brief Front Right Motor Input B
#define FRINB 26
//! @def FLPWM @brief Front Left Motor Power Pin
#define FLPWM 11
//! @def FLINA @brief Front Left Motor Input A
#define FLINA 24
//! @def FLINB @brief Front Left Motor Input B
#define FLINB 12
//! @def BRPWM @brief Back Right Motor Power Pin
#define BRPWM 28
//! @def BRINA @brief Back Right Motor Input A
#define BRINA 30
//! @def BRINB @brief Back Right Motor Input B
#define BRINB 29
//! @def BLPWM @brief Back Left Motor Power Pin
#define BLPWM 8
//! @def BLINA @brief Back Left Motor Input A
#define BLINA 10
//! @def BLINB @brief Back Left Motor Input B
#define BLINB 9

//! @def TSSPNUM @brief Number of Tssps on the robot
#define TSSPNUM 16
//! @def TSSP_DEVIATION_CONSTANT @brief Constant to multiply the deviation
//!                                     offset to calculate a more accurate
//!                                     reading.
#define TSSP_DEVIATION_CONSTANT 0
//! @def BALL_DIS_MULTIPLIER @brief Constant to multiply ball strength to
//!                                 return ball distance.
#define BALL_DIS_MULTIPLIER 1
//! @def BALL_DIS_MULTIPLIER @brief Used to scale ball strength to calculate
//!                                 speed.
#define BALL_STRENGTH_MULTIPLIER 1
//! @def TSSP1 @brief Tssp Pin
#define TSSP1 16
//! @def TSSP2 @brief Tssp Pin
#define TSSP2 13
//! @def TSSP3 @brief Tssp Pin
#define TSSP3 41
//! @def TSSP4 @brief Tssp Pin
#define TSSP4 40
//! @def TSSP5 @brief Tssp Pin
#define TSSP5 39
//! @def TSSP6 @brief Tssp Pin
#define TSSP6 37
//! @def TSSP7 @brief Tssp Pin
#define TSSP7 36
//! @def TSSP8 @brief Tssp Pin
#define TSSP8 33
//! @def TSSP9 @brief Tssp Pin
#define TSSP9 32
//! @def TSSP10 @brief Tssp Pin
#define TSSP10 7
//! @def TSSP11 @brief Tssp Pin
#define TSSP11 6
//! @def TSSP12 @brief Tssp Pin
#define TSSP12 5
//! @def TSSP13 @brief Tssp Pin
#define TSSP13 4
//! @def TSSP14 @brief Tssp Pin
#define TSSP14 3
//! @def TSSP15 @brief Tssp Pin
#define TSSP15 2
//! @def TSSP16 @brief Tssp Pin
#define TSSP16 17

//! @def NUM_LS @brief Number of light sensors
#define NUM_LS 32
//! @def NUMBER_MUX @brief Number of mux's
#define NUMBER_MUX 2
//! @def LIGHT_PIN @brief Light Sensor Pin
#define LIGHT_PIN 21 
//! @def LIGHT_PIN2 @brief Light Sensor Pin
#define LIGHT_PIN2 20
//! @def LIGHT_PIN_DIGI_0 @brief Light Sensor Pin in correspondance to Mux
#define LIGHT_PIN_DIGI_0 23
//! @def LIGHT_PIN_DIGI_1 @brief Light Sensor Pin in correspondance to Mux
#define LIGHT_PIN_DIGI_1 22
//! @def LIGHT_PIN_DIGI_2 @brief Light Sensor Pin in correspondance to Mux
#define LIGHT_PIN_DIGI_2 15
//! @def LIGHT_PIN_DIGI_3 @brief Light Sensor Pin in correspondance to Mux
#define LIGHT_PIN_DIGI_3 14

//! @def BAT_READ_PIN @brief Pin used to read battery of robot LiPo
#define BAT_READ_PIN 38

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

//! @def BLUETOOTH_SERIAL @brief Serial used to transfer data from the
//!                              bluetooth modules
#define BLUETOOTH_SERIAL Serial1
//! @def BLUETOOTH_BAUD @brief Bluetooth communication baud rate.
#define BLUETOOTH_BAUD 9600
//! @def BLUETOOTH_PACKET_SIZE @brief Size of data being sent over.
#define BLUETOOTH_PACKET_SIZE 5
//! @def BLUETOOTH_START_BYTE @brief Byte start identifier.
#define BLUETOOTH_START_BYTE 254
//! @def BLUETOOTH_NO_DATA @brief If module has no data output.
#define BLUETOOTH_NO_DATA 255   

#endif