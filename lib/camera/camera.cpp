/*!
 * @file Camera.cpp
 */
 
#include "camera.h"
 
// i think we need the UART Libary
 
/*!
 * @brief Initalizes The Camera System
 */
void Camera::init(){
    cameraSerial.begin(115200); // frequancey thing :D
}
/*!
 * @brief a function that comuniates to the camera and recieves its information
 * @return the values that the camera is transmitting [goal_x, goal_y, 
 *         goal_index (colour), angle_to_goal]
 */
void Camera::read_camera(){
    if (cameraSerial.available() >= 6) {
        // read the incoming stuff
        if(cameraSerial.read() == 200){
            if(cameraSerial.peek() == 122){
                cameraSerial.read();
                goal_x_yellow = cameraSerial.read();
                if(goal_x_yellow == 0) {
                    goal_x_yellow = goal_x_yellow;
                } else {
                    goal_x_yellow -= 60;
                }
                goal_y_yellow = cameraSerial.read();
                if(goal_y_yellow == 0) {
                    goal_y_yellow = goal_y_yellow;
                } else {
                    goal_y_yellow -= 60;
                }
                angle_to_goal_yellow = calculate_theta(goal_x_yellow,goal_y_yellow);
                goal_x_blue = cameraSerial.read();
                if(goal_x_blue == 0) {
                    goal_x_blue = goal_x_blue;
                } else {
                    goal_x_blue -= 60;
                }
                goal_y_blue = cameraSerial.read();
                if(goal_y_blue == 0) {
                    goal_y_blue = goal_y_blue;
                } else {
                    goal_y_blue -= 60;
                }
                angle_to_goal_blue = calculate_theta(goal_y_blue,goal_x_blue);
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
float Camera::calculate_theta(float o, float a){
    return 90 - (atan2(o, a) * RAD_TO_DEG);
}