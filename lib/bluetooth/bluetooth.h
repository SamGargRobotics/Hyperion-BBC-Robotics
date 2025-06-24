/*!
 * @file bluetooth.h
 * 
 * @mainpage This is a library for the HC-05 Bluetooth Module
 * 
 * @date 30/04/25
 * 
 * @author S.Garg (Brisbane Boys' College)
 */
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <common.h>

/*!
 * @brief Class that stores state and functions for interacting with the HC-05 
 *        Bluetooth Module
 */
class Bluetooth {
public:
    Bluetooth() {};
    void init();
    void update(bool logic, float ballDir, float ballDis);
    void send(bool logic, float ballDir, float ballDis);
    //! @brief Location of the ball for the connected robot. [1] is ballDir, [2]
    //!        is ballStr
    float otherRobotBallLocation[2] = {0,0};
    //! @brief Previous loop's other robot ball strength recorded.
    float prevBallStr = 0.00;
    //! @brief Previous loop's other robot logic recorded.
    bool prevAttacking = 0;
    //! @brief If the other robot is attacking or defending.
    float otherRobotLogic = 0;
    //! @brief If the robot is connected to another bluetooth module
    bool connection;
private:
    bool read();
    bool read_data;
    int bluetoothBuffer[BLUETOOTH_PACKET_SIZE - 1];
    unsigned long last_sent_time;
    unsigned long last_received_time;
};


#endif