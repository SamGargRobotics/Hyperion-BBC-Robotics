/*!
 * @file VoltDiv.cpp
 */
#include "VoltDiv.h"

/*!
 * @brief Initializes the pin to ensure that it can be read from.
 */
void VoltDiv::init() {
    pinMode(pin, INPUT);
}

/*! 
 * @brief Reads the voltage level through a voltage divider to recieve an analog
 *        value which can be converted to voltage via calculations.
 * 
 * @returns Voltage drawn through voltage divider in volts.
 */
float VoltDiv::update() {
    #if DEBUG_VD
        Serial.print("Analog: ");
        Serial.print(analogRead(pin));
        Serial.print(" Final: ");
        Serial.println(analogRead(pin) / divider);
    #endif
    return analogRead(pin) / divider;
}

bool VoltDiv::getMotorOn() {
    return motorOn;
}