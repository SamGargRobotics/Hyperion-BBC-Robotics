/*!
 * @file batread.h
 * 
 * @mainpage This is a library used to read the battery level of the robot.
 * 
 * @date 30/04/25
 * 
 * @author S.Garg (Brisbane Boys' College)
 */
#ifndef BATREAD_H
#define BATREAD_H

#include <Arduino.h>
#include <config.h>
#include <pins.h>

/*!
 * @brief Class that stores state and functions for interacting with the
 *        robot's electrical circuit using voltage dividers to uncover battery
 *        level.
 */
class BatRead { 
public:
    BatRead() {};
    void init();
    void read();
    float volts = 0;
    bool motorOn = false;
private:
    void calcBat(float rawVal);
    void calcSwitchStatus(float V);
    //! @brief Raw value read from the pin.
    float rawValue = 0;
};

#endif