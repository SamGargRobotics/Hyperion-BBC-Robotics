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
 
/*!
 * @brief Class that stores state and functions for reading what the camera is 
          transmitting
*/
class Camera {
public:
    void init();
    void read_camera();
    int goal_x_attack;
    int goal_x_defend;
    int goal_y_attack;
    int goal_y_defend;
    float angle_to_goal_attack;
    float angle_to_goal_defend;
private:
    float calculate_hypot(float x, float y);
    float calculate_theta(float o, float h);
    int goal_index_defend;
    int goal_index_attack;
};
 
#endif