/*!
 * @file batread.cpp
 */
#include "batread.h"

/*!
 * @brief Initializes the pin to ensure that it can be read from.
 */
void BatRead::init() {
    pinMode(BAT_READ_PIN, INPUT);
    pinMode(BAT_LED_PIN, OUTPUT);
}

/*!
 * @brief Reads the pin as an analog value to ensure that it can be used across
 *        the code.
 */
void BatRead::read() { 
    rawValue = analogRead(BAT_READ_PIN);
    calcBat(rawValue);
    calcSwitchStatus(volts);
    #if BAT_READ_RAWVAL
        Serial.println(rawValue);
    #endif
}

/*!
 * @brief Calculates battery in volts according to analog reading.
 * 
 * @param rawVal Analog value from the batread function
*/
void BatRead::calcBat(float rawVal) {
    volts = (rawVal-18.25)/70.5;
}

/*!
 * @brief Calculates whether the motor switch is on or off.
 * 
 * @param V Voltage of the Battery.
 */
void BatRead::calcSwitchStatus(float V) {
    motorOn = (V > BAT_MOTOROFF_THRESH);
}