/*!
 * @file common.h
 * 
 * This is a library of common information and settings used over multiple
 * libraries.
 * 
 * S.Garg (Brisbane Boys' College)
 * T.McCabe (Brisbane Boys' College)
 */
#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <math.h>

float floatMod(float x, float m);

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

//! @def TSSP1_X @brief X component on unit circle for Tssp 1
#define TSSP1_X 0
//! @def TSSP2_X @brief X component on unit circle for Tssp 2
#define TSSP2_X 0.382683432
//! @def TSSP3_X @brief X component on unit circle for Tssp 3
#define TSSP3_X 0.707106781
//! @def TSSP4_X @brief X component on unit circle for Tssp 4
#define TSSP4_X 0.923879533
//! @def TSSP5_X @brief X component on unit circle for Tssp 5
#define TSSP5_X 1
//! @def TSSP6_X @brief X component on unit circle for Tssp 6
#define TSSP6_X 0.923879533
//! @def TSSP7_X @brief X component on unit circle for Tssp 7
#define TSSP7_X 0.707106781
//! @def TSSP8_X @brief X component on unit circle for Tssp 8
#define TSSP8_X 0.382683432
//! @def TSSP9_X @brief X component on unit circle for Tssp 9
#define TSSP9_X 0
//! @def TSSP10_X @brief X component on unit circle for Tssp 10
#define TSSP10_X -0.382683432
//! @def TSSP11_X @brief X component on unit circle for Tssp 11
#define TSSP11_X -0.707106781
//! @def TSSP12_X @brief X component on unit circle for Tssp 12
#define TSSP12_X -0.923879533
//! @def TSSP13_X @brief X component on unit circle for Tssp 13
#define TSSP13_X -1
//! @def TSSP14_X @brief X component on unit circle for Tssp 14
#define TSSP14_X -0.923879533
//! @def TSSP15_X @brief X component on unit circle for Tssp 15
#define TSSP15_X -0.707106781
//! @def TSSP16_X @brief X component on unit circle for Tssp 16
#define TSSP16_X -0.382683432

//! @def TSSP1_Y @brief Y component on unit circle for Tssp 1
#define TSSP1_Y 1
//! @def TSSP2_Y @brief Y component on unit circle for Tssp 2
#define TSSP2_Y 0.923879533
//! @def TSSP3_Y @brief Y component on unit circle for Tssp 3
#define TSSP3_Y 0.707106781
//! @def TSSP4_Y @brief Y component on unit circle for Tssp 4
#define TSSP4_Y 0.382683432
//! @def TSSP5_Y @brief Y component on unit circle for Tssp 5
#define TSSP5_Y 0
//! @def TSSP6_Y @brief Y component on unit circle for Tssp 6
#define TSSP6_Y -0.382683432
//! @def TSSP7_Y @brief Y component on unit circle for Tssp 7
#define TSSP7_Y -0.707106781
//! @def TSSP8_Y @brief Y component on unit circle for Tssp 8
#define TSSP8_Y -0.923879533
//! @def TSSP9_Y @brief Y component on unit circle for Tssp 9
#define TSSP9_Y -1
//! @def TSSP10_Y @brief Y component on unit circle for Tssp 10
#define TSSP10_Y -0.923879533
//! @def TSSP11_Y @brief Y component on unit circle for Tssp 11
#define TSSP11_Y -0.707106781
//! @def TSSP12_Y @brief Y component on unit circle for Tssp 12
#define TSSP12_Y -0.382683432
//! @def TSSP13_Y @brief Y component on unit circle for Tssp 13
#define TSSP13_Y 0
//! @def TSSP14_Y @brief Y component on unit circle for Tssp 14
#define TSSP14_Y 0.382683432
//! @def TSSP15_Y @brief Y component on unit circle for Tssp 15
#define TSSP15_Y 0.707106781
//! @def TSSP16_Y @brief Y component on unit circle for Tssp 16
#define TSSP16_Y 0.923879533

//! @def FIRST_HIGHEST_TSSP_STR_MULTIPLIER @brief Multiplier to calculate 
//! advanced ball strength.
#define FIRST_HIGHEST_TSSP_STR_MULTIPLIER 6
//! @def SECOND_HIGHEST_TSSP_STR_MULTIPLIER @brief Multiplier to calculate 
//! advanced ball strength.
#define SECOND_HIGHEST_TSSP_STR_MULTIPLIER 4
//! @def THIRD_HIGHEST_TSSP_STR_MULTIPLIER @brief Multiplier to calculate 
//! advanced ball strength.
#define THIRD_HIGHEST_TSSP_STR_MULTIPLIER 3
//! @def FOURTH_HIGHEST_TSSP_STR_MULTIPLIER @brief Multiplier to calculate 
//! advanced ball strength.
#define FOURTH_HIGHEST_TSSP_STR_MULTIPLIER 2

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