/*!
 * @file Camera.cpp
 *
 * @mainpage Camera calculations for Robot and Goal Tracking
 *
 * This is a library for Camera calculations within the robot's code.
*/
 
#include "camera.h"
#include <math.h>
 
// i think we need the UART Libary
 
/*!
 * @brief Initalizes The Camera System
*/
void Camera::init_camera(){
    Serial1.begin(115200); // frequancey thing :D
}
/*!
 * @brief a function that comuniates to the camera and recieves its information
 * @return the values that the camera is transmitting [goal_x, goal_y, goal_index (colour), angle_to_goal]
*/
void Camera::read_camera(){
    if (Serial.available() >= 5) {
        // read the incoming stuff
        if(Serial1.read() == 200){
            if(Serial1.peek() == 122){
                Serial.read();
                goal_x = Serial1.read();
                goal_y = Serial1.read();
                goal_index = Serial1.read(); //goal color
                angle_to_goal = calculate_theta(goal_x,goal_y);
                return [angle_to_goal,goal_index];
            }
        }
    }
}
 
/*!
 * @brief calculate the hypotinus from two distances
 * @return returns the hypotinus
*/
float Camera::calculate_hypot(float x, float y){
    return sqrt(pow(x,2) + pow(y,2));
}
 
/*!
 * @brief calculate the angle from two distances
 * @return returns theta
*/
float Camera::calculate_theta(float o, float h){
    return acosf(o/h);
}