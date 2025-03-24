/*!
 * @file batread.cpp
 * 
 * @mainpage Voltage Dividers
 * 
 * This is a library to read the robot's battery level using voltage dividers. 
*/
#include "batread.h"

/*!
 * @brief Initializes the pin to ensure that it can be read from.
*/
void BatRead::init() {
    pinMode(BAT_READ_PIN, INPUT);
}

/*!
 * @brief Reads the pin as an analog value to ensure that it can be used across
 *        the code.
*/
float BatRead::read() { 
    rawValue = analogRead(BAT_READ_PIN);
    float scaledValue = ((rawValue - lowest) / (highest - lowest)) * 100;
    return scaledValue; //returns a value from 0-100.
}