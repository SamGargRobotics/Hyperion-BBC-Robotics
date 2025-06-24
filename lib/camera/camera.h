/*!
 * @file LSLB.h
 *
 * @mainpage This is a library to read what the camera is transmitting for goal 
 * tracking
 * 
 * @date 30/04/25
 *
 * @author T.McCabe (Brisbane Boys' College)
*/
#ifndef CAMERA_H
#define CAMERA_H
 
#include <Arduino.h>
#include <math.h>
#include <config.h>
#include <common.h>
 
/*!
 * @brief Class that stores state and functions for reading what the camera is 
          transmitting
*/
class Camera {
public:
    void init();
    void read_camera();
    //! @brief x value corresponding to the yellow goal
    int goal_x_yellow;
    //! @brief x value corresponding to the blue goal
    int goal_x_blue;
    //! @brief y value corresponding to the yellow goal
    int goal_y_yellow;
    //! @brief y value corresponding to the blue goal
    int goal_y_blue;
    //! @brief Previous y and x values for corresponding goals
    int previousVals[4] = {-1};
    //! @brief Angle to the yellow goal
    float angle_to_goal_yellow;
    //! @brief Angle to the blue goal
    float angle_to_goal_blue;
    //! @brief Blob area of yellow goal
    float distYel;
    //! @brief Blob area of blue goal
    float distBlue;
private:
    float calculate_hypot(float x, float y);
    float calculate_theta(float o, float h);
};
 
#endif