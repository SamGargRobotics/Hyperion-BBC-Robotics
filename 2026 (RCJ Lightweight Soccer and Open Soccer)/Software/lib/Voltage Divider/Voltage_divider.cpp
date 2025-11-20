/*!
 * @file VoltDiv.cpp
 */
#include "Voltage_divider.h"

/*!
 * @brief Initializes the pin to ensure that it can be read from.
 */
void VoltageDivider::init() {
    pinMode(pin, INPUT);
}

/*! 
 * @brief Reads the voltage level through a voltage divider to recieve an analog
 *        value which can be converted to voltage via calculations.
 * 
 * @returns Voltage drawn through voltage divider in volts.
 */
void VoltageDivider::update() {
    #if DEBUG_VD
        Serial.print("Analog: ");
        Serial.print(analogRead(pin));
        Serial.print(" Final: ");
        Serial.println(analogRead(pin) / divider);
    #endif
    lvl = analogRead(pin) / divider;
}

float VoltageDivider::get_lvl() {
    return lvl;
}

bool VoltageDivider::get_motor_on() {
    return motorOn;
}