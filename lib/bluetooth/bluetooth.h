/*!
 * @file bluetooth.h
 * 
 * This is a library for the HC-05 Bluetooth Module
 * 
 * S.Garg (Brisbane Boys' College)
 */
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <configandpins.h>

/*!
 * @brief Class that stores state and functions for interacting with the HC-05 
 *        Bluetooth Module
 */
class Bluetooth {
public:
    Bluetooth() {};
    void init();
    void update(float batLevel, float ballDir, float ballDis);
    void send(float batLevel, float ballDir, float ballDis);
    float otherRobotBallLocation[2] = {0,0};
    float otherRobotBatteryLevel = 0;
    bool connection;
private:
    bool read();
    bool read_data;
    int bluetoothBuffer[BLUETOOTH_PACKET_SIZE - 1];
    unsigned long last_sent_time;
    unsigned long last_received_time;
};


#endif