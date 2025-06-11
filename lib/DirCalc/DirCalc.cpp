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
float DirectionCalc::calcSpeed(float ballStr, float ballDir) {
    #if SECOND_ROBOT
        multi = (5*SET_SPEED/8);
        steepness = 97.5;
    #else
        multi = (3*SET_SPEED/4);
        steepness = 90.5;
    #endif
    if(ballDir >= 90 && ballDir <= 270) {
        return max(min(pow(EULER, -0.02*(ballStr-(steepness*EULER))) + 30, \
            multi), 30)/ multi;  
    } else {
        return max(min(pow(EULER, -0.02*(ballStr-(steepness*EULER))) + 20 , \
            multi), 30)/multi;
    }

}