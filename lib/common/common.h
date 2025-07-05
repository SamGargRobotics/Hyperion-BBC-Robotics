/*!
 * @file common.h
 * 
 * @mainpage This is a library of common information and settings used over 
 * multiple libraries.
 * 
 * @date 30/04/25
 * 
 * @author S.Garg (Brisbane Boys' College)
 * @author T.McCabe (Brisbane Boys' College)
 */
#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <math.h>

float floatMod(float x, float m);
float angleDiff(float a, float b);
float circularDiff(float a, float b);

#define ARRAYSHIFTDOWN(a, lower, upper){          \
	if (upper == (sizeof(a)/sizeof(a[0])) - 1){   \
		for (int q = upper - 1; q >= lower; q--){ \
			*(a + q + 1) = *(a + q); }            \
	} else{                                       \
		for (int q = upper; q >= lower; q--){     \
			*(a + q + 1) = *(a + q); }}}

//! @def MOTORNUM @brief Number of motors.
#define MOTORNUM 4

//! @def TSSPNUM @brief Number of Tssps on the robot
#define TSSPNUM 16

//! @def NUM_LS @brief Number of light sensors
#define NUM_LS 32
//! @def NUMBER_MUX @brief Number of mux's
#define NUMBER_MUX 2

//! @def PI @brief π
#define PI 3.1415926535897932384626433832795
//! @def HALF_PI @brief π/2
#define HALF_PI 1.5707963267948966192313216916398
//! @def TWO_PI @brief 2π
#define TWO_PI 6.283185307179586476925286766559
//! @def DEG_TO_RAD @brief Degrees to Radians
#define DEG_TO_RAD 0.017453292519943295769236907684886
//! @def RAD_TO_DEG @brief Radians to Degrees
#define RAD_TO_DEG 57.295779513082320876798154814105
//! @def EULER @brief Euler's Number
#define EULER 2.718281828459045235360287471352
//! @def CIRCLE_DEGREES @brief Number of degrees in a circle
#define CIRCLE_DEGREES 360
//! @def SEMI_CIRCLE_DEGREES @brief Number of degrees in a semi circle
#define SEMI_CIRCLE_DEGREES 180

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

//! @def cameraSerial @brief Serial that the camera sends data over to
#define cameraSerial Serial8

//! @def BLUETOOTH_SERIAL @brief Serial used to transfer data from the
//!                              bluetooth modules
#define BLUETOOTH_SERIAL Serial1
//! @def BLUETOOTH_BAUD @brief Bluetooth communication baud rate.
#define BLUETOOTH_BAUD 9600
//! @def BLUETOOTH_PACKET_SIZE @brief Size of data being sent over.
#define BLUETOOTH_PACKET_SIZE 4
//! @def BLUETOOTH_START_BYTE @brief Byte start identifier.
#define BLUETOOTH_START_BYTE 254
//! @def BLUETOOTH_NO_DATA @brief If module has no data output.
#define BLUETOOTH_NO_DATA 255


#endif