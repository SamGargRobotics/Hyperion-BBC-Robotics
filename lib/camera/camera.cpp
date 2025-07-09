/*!
 * @file Camera.cpp
 */
 
#include "Camera.h"
 
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
void Camera::update(bool attackBlue){
    if (cameraSerial.available() >= CAM_PACKET_SIZE) {
        // read the incoming stuff
        uint8_t byte1 = cameraSerial.read();
        uint8_t byte2 = cameraSerial.peek();
        if(byte1 == CAM_START_PACK_1 && byte2 == CAM_START_PACK_2) {
            cameraSerial.read();
            uint8_t goal_x_yellow = cameraSerial.read();
            uint8_t goal_y_yellow = cameraSerial.read();
            uint8_t goal_x_blue = cameraSerial.read();
            uint8_t goal_y_blue = cameraSerial.read();
            if(goal_x_yellow != 255) {
                goal_x_yellow -= 60;
                goal_y_yellow -= 60;
            }
            if(goal_x_blue != 255) {
                goal_x_blue -= 60;
                goal_y_yellow -= 60;
            }
            if (attackBlue) {
                attackGoalAngle = calculate_theta(goal_y_blue, goal_x_blue);
                attackGoalDist = calculate_hypot(goal_x_blue, goal_y_blue);
                defendGoalAngle = calculate_theta(goal_y_yellow, goal_x_yellow);
                defendGoalDist = calculate_hypot(goal_x_yellow, goal_y_yellow);
                seeingAttackingGoal = (goal_y_blue != 255);
                seeingDefendingGoal = (goal_y_yellow != 255);
            } else {
                attackGoalAngle = calculate_theta(goal_y_yellow, goal_x_yellow);
                attackGoalDist = calculate_hypot(goal_x_yellow, goal_y_yellow);
                defendGoalAngle = calculate_theta(goal_y_blue, goal_x_blue);
                defendGoalDist = calculate_hypot(goal_x_blue, goal_y_blue);
                seeingAttackingGoal = (goal_y_yellow != 255);
                seeingDefendingGoal = (goal_y_blue != 255);
            }
        }
    }
}

float Camera::getAttackGoalAngle() {
    return attackGoalAngle;
}

float Camera::getAttackGoalDist() {
    return attackGoalDist;
}

float Camera::getDefendGoalAngle() {
    return defendGoalAngle;
}

float Camera::getDefendGoalDist() {
    return defendGoalDist;
}

bool Camera::getAttackGoalVisible() {
    return seeingAttackingGoal;
}

bool Camera::getDefendGoalVisible() {
    return seeingDefendingGoal;
}

/*!
 * @brief Calculates the hypotenuse on a triangle based on two distances
 * @return Returns the Hypotenuse
 */
float Camera::calculate_hypot(float x, float y){
    return sqrtf(pow(x,2) + pow(y,2));
}
 
/*!
 * @brief Calculates the angle between two distances using trigonometry.
 * @return Returns Theta
 */
float Camera::calculate_theta(float opp, float adj){
    return 90 - (atan2f(opp, adj) * RAD_TO_DEG);
}