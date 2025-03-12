#include "camera.h"
#include <math.h>

// i think we need the UART Libary

void Camera::init_camera(){
    Serial1.begin(115200); // frequancey thing :D
}

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

float Camera::calculate_hypot(float x, float y){
    return sqrt(pow(x,2) + pow(y,2));
}

float Camera::calculate_theta(float o, float h){
    return acosf(o/h);
}