/*!
 * @file LSLB.h
 *
 * This is a library to read what the camera is transmitting for goal tracking
 *
 * T.McCabe (Brisbane Boys' College)
*/
#ifndef CAMERA_H
#define CAMERA_H
 
#include <Arduino.h>
#include <math.h>
#include <config.h>
 
/*!
 * @brief Class that stores state and functions for reading what the camera is 
          transmitting
*/
class Camera {
public:
    void init();
    void read_camera();
    int goal_x_yellow;
    int goal_x_blue;
    int goal_y_yellow;
    int goal_y_blue;
    float angle_to_goal_yellow;
    float angle_to_goal_blue;
private:
    float calculate_hypot(float x, float y);
    float calculate_theta(float o, float h);
};
 
#endif