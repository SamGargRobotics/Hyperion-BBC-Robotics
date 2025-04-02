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
void Camera::init(){
    Serial1.begin(115200); // frequancey thing :D
}
/*!
 * @brief a function that comuniates to the camera and recieves its information
 * @return the values that the camera is transmitting [goal_x, goal_y, 
 *         goal_index (colour), angle_to_goal]
*/
void Camera::read_camera(){
    if (Serial.available() >= 5) {
        // read the incoming stuff
        if(Serial1.read() == 200){
            if(Serial1.peek() == 122){
                Serial.read();
                goal_x_attack = Serial1.read();
                goal_y_attack = Serial1.read();
                goal_index_attack = Serial1.read(); //goal color
                angle_to_goal_attack = calculate_theta(goal_x_attack,goal_y_attack);
                if(Serial1.peek() == 202){
                    Serial.read();
                    goal_x_defend = Serial1.read();
                    goal_y_defend = Serial1.read();
                    goal_index_defend = Serial1.read(); //goal color
                    angle_to_goal_defend = calculate_theta(goal_x_defend,goal_y_defend);
                }
            }
        }
    }
   
    goal_x_defend = Serial1.read();
    goal_y_defend = Serial1.read();
    goal_index_defend = Serial1.read(); //goal color
    angle_to_goal_defend = calculate_theta(goal_x_defend,goal_y_defend);
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