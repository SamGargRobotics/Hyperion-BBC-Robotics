/*!
 * @file DirCalc.cpp
 */
#include "DirCalc.h"

/*!
 * @brief Calculates movement direction using an exponential equation.
 * 
 * @param ballDir Current ball direction.
 * @param ballStr Current ball strength.
 * 
 * @return Robot movement angle.
 */
float DirectionCalc::exponentialOrbit(float ballDir, float ballStr) {
    // Standard exponential orbit, to be initially used for debugging and 
    // simpler versions of orbitting.
    // Find our exponential graph here: 
    // https://www.desmos.com/calculator/mjjqu8ujy0
    if(ballDir > 180) {
        modBallDir = ballDir-360;
        return ballDir + (-1*min(0.04*(pow(EULER, -4.5*modBallDir) - 1), \
                                 EXPO_MIN_VAL));

    } else {
        return ballDir + (min(0.04*(pow(EULER, 4.5*ballDir) - 1), EXPO_MIN_VAL));
    }
}

/*!
 * @brief Main code for the robot that determines it's current positioning and 
 *        the direction it needs to move in
 * 
 * @param goalDir Direction of the goal relative to the robot.
 * @param goalDis Distance away from the goal relative to the robot.
 * @param ballDir Current direction of the ball.
 * 
 * @return Robot movement angle.
 */
int DirectionCalc::defenderMovement(float goalDir, float goalDis, float ballDir) 
    {
    // Determine forward or backward movement relative to the semi-circle
    if(goalDis > GOAL_SEMI_CIRCLE_RADIUS_CM) {
        defenderMoveDir = 180;
    } else if(goalDis < GOAL_SEMI_CIRCLE_RADIUS_CM) {
        defenderMoveDir = 0;
    } else {
        defenderMoveDir = -1;
    }
    // Return movement angle based on direction
    if(defenderMoveDir != -1) {
        return abs(ballDir - defenderMoveDir);
    } else {
        return defenderMoveDir;
    }
}

/*!
 * @brief Finds the middle of two angles, used in the defender code to find the
 *        middle value between goal and ball direction.
 * 
 * @param angle1 The first angle used to find the middle angle.
 * @param angle2 The second angle used to find the middle angle.
 * 
 * @return Middle angle of the two angles provided in perameters.
 */
double DirectionCalc::findMiddleAngle(double angle1, double angle2) {
    // Convert angles to radians
    double angle1_rad = angle1 * M_PI / 180.0;
    double angle2_rad = angle2 * M_PI / 180.0;

    // Calculate the Cartesian coordinates (x, y)
    double x = cos(angle1_rad) + cos(angle2_rad);
    double y = sin(angle1_rad) + sin(angle2_rad);

    // Calculate the middle angle using atan2
    double middle_angle_rad = atan2(y, x);

    // Convert the result back to degrees
    double middle_angle_deg = middle_angle_rad * 180.0 / M_PI;

    // Normalize the angle to be between 0 and 360 degrees
    if (middle_angle_deg < 0) {
        middle_angle_deg += 360.0;
    }

    return middle_angle_deg;
}

/*!
 * @brief Calculate movement speed based on the distance of the ball. The
 *        further away the ball is, the faster the robot must move. The closer 
 *        the robot is, the slower it is. However, if the ball is directly
 *        infront of the robot, it will travel at full speed (logic main)
 *        https://www.desmos.com/calculator/irtx2usk2w
 * 
 * @param ballStr Strength of ball away from the robot
 * 
 * @return Returns a speed value for the robot to move.
 */
float DirectionCalc::calcSpeed(float ballStr) {
    return max(min(pow(EULER, -0.02*(ballStr-(90.5*EULER))) + 20, \
           ((3*SET_SPEED)/4)), 30)/((3*SET_SPEED)/4);
}