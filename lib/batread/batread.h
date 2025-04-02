/*!
 * @file batread.h
 * This is a library used to read the battery level of the robot.
 * 
 * S.Garg (Brisbane Boys' College)
 */
#ifndef BATREAD_H
#define BATREAD_H

#include <Arduino.h>
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
    float read();
private:
    float rawValue = 0;
    float highest = 2.9302325581;
    float lowest = 2.651167907;
};

#endif