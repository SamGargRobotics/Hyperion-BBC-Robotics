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

float floatMod(float x, float m);

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