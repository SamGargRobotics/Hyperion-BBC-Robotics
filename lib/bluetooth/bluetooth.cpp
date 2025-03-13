/*!
 * @file bluetooth.cpp
 * 
 * @mainpage HC-05 Bluetooth Module
 * 
 * This is a library for reading and sending values accross a HC-05 Bluetooth
 * Module
*/
#include <bluetooth.h>

/*!
 * @brief Initializes the module for use accross the library and code
*/
void Bluetooth::init() {
    BLUETOOTH_SERIAL.begin(BLUETOOTH_BAUD);
    last_received_time = micros();
    last_sent_time = micros();
}