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
int intMod(int x, int m);
float circularDiff(float a, float b);
float angleBetween(float angleCounterClockwise, float angleClockwise);
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise);
float midAngleBetween(float angleCounterClockwise, float angleClockwise);

/**
 * @brief Shifts elements in an array down by one position within a specified range.
 *
 * This macro shifts the elements in array `a` one index forward (to the right), starting from
 * index `lower` to `upper`, inclusive. It performs a bounds-aware shift, handling the case
 * where `upper` is the last index in the array.
 *
 * **Usage Note:** The macro relies on `sizeof(a)/sizeof(a[0])` to determine the array size,
 * so `a` must be a true array (not a pointer). Be cautious when using this with dynamically
 * allocated arrays or function parameters.
 *
 * @param a The array whose elements are to be shifted.
 * @param lower The starting index of the range to shift (inclusive).
 * @param upper The ending index of the range to shift (inclusive).
 */
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

//! @def BT_SERIAL @brief Serial used to transfer data from the
//!                              bluetooth modules
#define BT_SERIAL Serial1
//! @def BT_BAUD @brief Bluetooth communication baud rate.
#define BT_BAUD 9600
//! @def BT_PACKET_SIZE @brief Size of data being sent over.
#define BT_PACKET_SIZE 4
//! @def BT_START_BYTE @brief Byte start identifier.
#define BT_START_BYTE 255
//! @def BT_NO_DATA @brief If module has no data output.
#define BT_NO_DATA 255


#endif