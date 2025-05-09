/*!
 * @file DirCalc.cpp
 */
#include "DirCalc.h"

/*! 
 * @brief Uses trigonometry to find the movement angle using associated params.
 * 
 * @param ballStr Distance of ball away from the robot.
 * @param ballDir Direction of ball relative to front of robot. (degrees)
 * 
 * @return Robot movement angle.
 */
float DirectionCalc::trigOrbit(float ballStr, float ballDir) {
    // Check for any exceptions to the orbit (that wont work with the regular 
    // calculations)
    if(ballDir >= 1 && ballDir <= 44) {
        // Calculating for the case that the ball is between 1 to 44 degrees 
        // relative to the robot (in direction)
        mainAngle = ballDir;
        lateralMoveDis = ballDis*sinf(mainAngle);
        verticalMoveDis = ballDis*cosf(mainAngle);
        secondaryAngle = atanf(decreaseVerticalDis(
            verticalMoveDis)/lateralMoveDis);
        moveAngle = 180 - (90+secondaryAngle);
    } else if(ballDir >= 46 && ballDir <= 89) {
        // Calculating for the case that the ball is between 46 to 89 degrees 
        // relative to the robot (in direction)
        mainAngle = 90 - ballDir;
        lateralMoveDis = ballDis*cosf(mainAngle);
        verticalMoveDis = ballDis*sinf(mainAngle);
        secondaryAngle = atanf(decreaseVerticalDis(
            verticalMoveDis)/lateralMoveDis);
        moveAngle = 90 - secondaryAngle;
    } else if(ballDir >= 91 && ballDir <= 134) {
        // Calculating for the case that the ball is between 91 to 134 degrees 
        // relative to the robot (in direction)
        mainAngle = ballDir - 90; //!! Make more efficient way of calculating
        lateralMoveDis = ballDis*cosf(mainAngle);
        verticalMoveDis = ballDis*sinf(mainAngle);
        secondaryAngle = atanf(increaseVerticalDis(
            verticalMoveDis)/lateralMoveDis);
        moveAngle = secondaryAngle + 90;
    } else if(ballDir >= 136 && ballDir <= 179) {
        // Calculating for the case that the ball is between 136 to 179 degrees 
        // relative to the robot (in direction)
        mainAngle = 180 - ballDir;
        lateralMoveDis = ballDis*sinf(mainAngle);
        verticalMoveDis = ballDis*cosf(mainAngle);
        secondaryAngle = atanf(increaseVerticalDis(
            verticalMoveDis) / lateralMoveDis);
        moveAngle = 180 - secondaryAngle;
    } else if(ballDir >= 181 && ballDir <= 214) {
        // Calculating for the case that the ball is between 181 to 214 degrees 
        // relative to the robot (in direction)
        mainAngle = ballDir - 180;
        lateralMoveDis = ballDis*sinf(mainAngle);
        verticalMoveDis = ballDis*cosf(mainAngle);
        secondaryAngle = atanf(increaseVerticalDis(
            verticalMoveDis)/lateralMoveDis);
        moveAngle = secondaryAngle + 180;
    } else if(ballDir >= 216 && ballDir <= 269) {
        // Calculating for the case that the ball is between 216 to 269 degrees 
        // relative to the robot (in direction)
        mainAngle = 270 - ballDir;
        lateralMoveDis = ballDis*cosf(mainAngle);
        verticalMoveDis = ballDis*sinf(mainAngle);
        secondaryAngle = atanf(increaseVerticalDis(
            verticalMoveDis)/lateralMoveDis);
        moveAngle = 270 - secondaryAngle;
    } else if(ballDir >= 271 && ballDir <= 314) {
        // Calculating for the case that the ball is between 271 to 314 degrees 
        // relative to the robot (in direction)
        mainAngle = ballDir - 270;
        lateralMoveDis = ballDis*cosf(mainAngle);
        verticalMoveDis = ballDis*sinf(mainAngle);
        secondaryAngle = atanf(decreaseVerticalDis(
            verticalMoveDis)/lateralMoveDis);
        moveAngle = secondaryAngle + 270;
    } else if(ballDir >= 316 && ballDir <= 359) {
        // Calculating for the case that the ball is between 316 to 359 degrees 
        // relative to the robot (in direction)
        mainAngle = 360 - ballDir;
        lateralMoveDis = ballDis*sinf(mainAngle);
        verticalMoveDis = ballDir*cosf(mainAngle);
        secondaryAngle = atanf(decreaseVerticalDis(
            verticalMoveDis)/lateralMoveDis);
        moveAngle = 360 - secondaryAngle;
    } else {
        // If the direction of the ball did not meet any of the above cases 
        // (0, 45, 90, 135, 180, 225, 270, 315) then they are covered below.
        for(uint8_t i = 0; i < 8; i++) {
            if(ballDir == StandardCases[i]) {
                standardCaseNum = i;
                if(ballDir <= 135) {
                    moveAngle = StandardCases[standardCaseNum] + 10;
                } else if(ballDir >= 215) {
                    moveAngle = StandardCases[standardCaseNum] - 10;
                } else {
                    if(moveAngle <= 180) {
                        moveAngle = 135;
                    } else {
                        moveAngle = 215;
                    }
                }
            }
        }
    }
    return moveAngle;
}

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
        // return ballDir - min(0.04*pow(ORBIT_MULTIPLIER, 4.5*ballDir), 
        //                     EXPO_MIN_VAL);

        return ballDir + (-1*min(0.04*(pow(EULER, -4.5*modBallDir) - 1), 60));

    } else {
        // return ballDir + min(0.04*pow(ORBIT_MULTIPLIER, 4.5*ballDir), 
        //                     EXPO_MIN_VAL);

        return ballDir + (min(0.04*(pow(EULER, 4.5*ballDir) - 1), 60));
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
    if (goalDis > GOAL_SEMI_CIRCLE_RADIUS_CM) {
        goalDisRelativeDirection = -1;
        goalDisCalculatedRelativeDirection = 180; // Move Backwards
    } else if (goalDis < GOAL_SEMI_CIRCLE_RADIUS_CM) {
        goalDisRelativeDirection = 1;
        goalDisCalculatedRelativeDirection = 0;  // Move Forwards
    } else {
        goalDisRelativeDirection = 0; // Stay Still
    }

    // Determine if the defender should move
    defenderMoving = (goalDisRelativeDirection == 0 && \
                     ballRobotRelativeDirection == 0);
    
    // Return movement angle based on direction
    if(goalDisCalculatedRelativeDirection != -1) {
        defenderMoving = true;
        return findMiddleAngle(ballDir, goalDisCalculatedRelativeDirection);
    } else {
        defenderMoving = false;
        return -1;
    }
}

/*!
 * @brief Calculates the rotation of the defender in accordance to the ball 
 *        direction
 * 
 * @param goalDir Current direction of the goal.
 */
void DirectionCalc::defenderRotCalc(float goalDir) {
    defenderRotationOffset = 90 - goalDir;
    defenderRotationOffset = constrain(defenderRotationOffset, -90, 90);
}

/*!
 * @brief Calculates whether local robot is attacking or defending.
 * 
 * @param externalBallDis The ball distance away from the other robot.
 * @param ballDis The ball distance away from the local robot.
 * 
 * @return True if attack, false if defense.
 */
bool DirectionCalc::calculateStrategy(float externalBallDis, float ballDis) {
    if(ballDis < externalBallDis) {
        return true;
    } else {
        return false;
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
 *        infront of the robot, it will travel at full speed.
 * 
 * @param ballStr_ratio Normalized ball strenght (0.92-0.97)
 * @param voltage Battery voltage (11-12.6V)
 * 
 * @return Returns a speed value for the robot to move.
 */
float DirectionCalc::calcSpeed(float ballStr) {
    return max(min(pow(EULER, -0.02*(ballStr-(96.5*EULER))) + 20, 100), 30)/100;
}

/*!
 * @brief Calculates the ball distance using a constant and ball strength.
 * 
 * @param ballStr Distance away from the ball.
 * 
 * @return Ball Distance away from the ball (cm).
 */
float DirectionCalc::ballDisScale(float ballStr) { 
    return ballStr*BALL_DIS_MULTIPLIER; //TUNE THE MULTIPLCATION BY THE CONSTANT
}

/*!
 * @brief Scale for the trigonometry orbit. (Decreasing factor)
 * 
 * @param verticalDis Vertical distance of the ball away from the ball.
 * 
 * @return Scaled vertical distance away from the ball in terms of robot.
 */
float DirectionCalc::decreaseVerticalDis(float verticalDis) {
    return (1 * verticalDis + 0); //LINEAR FUNCTION - NEEDS TUNING
}

/*!
 * @brief Scale for the trigonmetry orbit. (Increasing factor)
 * 
 * @param verDis Vertical distance of the ball away from the ball.
 * 
 * @return Scaled vertical distance away from the bal in terms of robot.
 */
float DirectionCalc::increaseVerticalDis(float verDis) {
    return (1 * verDis + 0);  //LINEAR FUNCTION - NEEDS TUNING
}