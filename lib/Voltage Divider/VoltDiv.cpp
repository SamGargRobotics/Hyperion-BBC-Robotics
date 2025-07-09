/*!
 * @file batread.cpp
 */
#include "VoltDiv.h"

/*!
 * @brief Initializes the pin to ensure that it can be read from.
 */
void VoltDiv::init() {
    pinMode(pin, INPUT);
}

float VoltDiv::update() {
    return analogRead(pin) / divider;
}