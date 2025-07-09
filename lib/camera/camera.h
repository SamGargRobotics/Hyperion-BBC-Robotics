/*!
 * @file LSLB.h
 *
 * @mainpage This is a library to read what the camera is transmitting for goal 
 * tracking
 * 
 * @date 30/04/25
 *
 * @author T.McCabe (Brisbane Boys' College)
*/
#ifndef CAMERA_H
#define CAMERA_H
 
#include <Arduino.h>
#include <math.h>
#include <config.h>
#include <common.h>
 
/*!
 * @brief Class that stores state and functions for reading what the camera is 
          transmitting
*/
class Camera {
public:
    void init();
    void update(bool attackBlue);

    float getAttackGoalAngle();
    float getAttackGoalDist();
    float getDefendGoalAngle();
    float getDefendGoalDist();
    bool getAttackGoalVisible();
    bool getDefendGoalVisible();

private:
    float attackGoalAngle;
    float attackGoalDist;
    float defendGoalAngle;
    float defendGoalDist;
    bool seeingAttackingGoal;
    bool seeingDefendingGoal;

    float calculate_hypot(float x, float y);
    float calculate_theta(float opp, float adj);
};
 
#endif