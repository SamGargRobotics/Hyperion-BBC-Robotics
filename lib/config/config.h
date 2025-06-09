/*!
 * @file config.h
 * 
 * @mainpage This is a library that includes variables and defines that change 
 * the code's behaviour.
 * 
 * @date 09/05/25
 * 
 * @author S.Garg (Brisbane Boys' College)
 * @author T.McCabe (Brisbane Boys' College)
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <common.h>

// --[MASTER Values]--
    //! @def targetGoal @brief Blue = 1, Yelow = 0; Assigns which goal is the target
    //!                        for either attacking or defending
    #define targetGoal 0
    //! @def SET_SPEED @brief Speed that is set for running
    #define SET_SPEED 180
    //! @def SECOND_ROBOT @brief Defines if the second or first robot is being tuned
    #define SECOND_ROBOT 1

// --[SURGE STATE Values]--
struct surgeState {
    bool surgeQ;
    uint32_t startMillis;
};

// --[PID Values]--
#if not SECOND_ROBOT
    //! @def PID_p_attack @brief Proportional aspect of PID
    #define PID_p_attack 0.9
    //! @def PID_i_attack @brief Intergral aspect of PID
    #define PID_i_attack 0
    //! @def PID_d_attack @brief Derivative aspect of PID
    #define PID_d_attack 0.055
    //! @def PID_p_defend @brief Proportional aspect of PID
    #define PID_p_defend 0.65
    //! @def PID_i_defend @brief Intergral aspect of PID
    #define PID_i_defend 0
    //! @def PID_d_defend @brief Derivative aspect of PID
    #define PID_d_defend 0.03
    //! @def PID_p_defender_movement_vert @brief Proportional aspect of PID
    #define PID_p_defender_movement_vert 5
    //! @def PID_i_defender_movement_vert @brief Intergral aspect of PID
    #define PID_i_defender_movement_vert 0
    //! @def PID_d_defender_movement_vert @brief Derivative aspect of PID
    #define PID_d_defender_movement_vert 0
    //! @def PID_p_defender_movement_hozt @brief Proportional aspect of PID
    #define PID_p_defender_movement_hozt 1.2
    //! @def PID_i_defender_movement_hozt @brief Intergral aspect of PID
    #define PID_i_defender_movement_hozt 0
    //! @def PID_d_defender_movement_hozt @brief Derivative aspect of PID
    #define PID_d_defender_movement_hozt 0
#else
    //! @def PID_p_attack @brief Proportional aspect of PID
    #define PID_p_attack 0.9
    //! @def PID_i_attack @brief Intergral aspect of PID
    #define PID_i_attack 0
    //! @def PID_d_attack @brief Derivative aspect of PID
    #define PID_d_attack 0.0275
    //! @def PID_p_defend @brief Proportional aspect of PID
    #define PID_p_defend 0.65
    //! @def PID_i_defend @brief Intergral aspect of PID
    #define PID_i_defend 0
    //! @def PID_d_defend @brief Derivative aspect of PID
    #define PID_d_defend 0.03
    //! @def PID_p_defender_movement_vert @brief Proportional aspect of PID
    #define PID_p_defender_movement_vert 3
    //! @def PID_i_defender_movement_vert @brief Intergral aspect of PID
    #define PID_i_defender_movement_vert 0
    //! @def PID_d_defender_movement_vert @brief Derivative aspect of PID
    #define PID_d_defender_movement_vert 0
    //! @def PID_p_defender_movement_hozt @brief Proportional aspect of PID
    #define PID_p_defender_movement_hozt 1.5
    //! @def PID_i_defender_movement_hozt @brief Intergral aspect of PID
    #define PID_i_defender_movement_hozt 0
    //! @def PID_d_defender_movement_hozt @brief Derivative aspect of PID
    #define PID_d_defender_movement_hozt 0
#endif
//! @def PID_abs_max @brief Absoloute max of PID
#define PID_abs_max SET_SPEED

// --[ATTACK LOGIC Values]
#if not SECOND_ROBOT
    //! @def ORBIT_STRENGTH_RADIUS @brief The strength value that the robot switches
    // strats for orbit
    #define ORBIT_STRENGTH_RADIUS 85
    //! @def SURGE_STR_VALUE @brief Min Strength value that allows the robot to 
    //!                             surge (Attack)
    #define SURGE_STR_VALUE 107
    //! @def EXPO_MIN_VAL @brief Minimum value of the exponential orbit
    #define EXPO_MIN_VAL 60
#else
    //! @def ORBIT_STRENGTH_RADIUS @brief The strength value that the robot switches
    // strats for orbit
    #define ORBIT_STRENGTH_RADIUS 100
    //! @def SURGE_STR_VALUE @brief Min Strength value that allows the robot to 
    //!                             surge (Attack)
    #define SURGE_STR_VALUE 110
    //! @def EXPO_MIN_VAL @brief Minimum value of the exponential orbit
    #define EXPO_MIN_VAL 60
#endif

// --[DEFENCE LOGIC Values]
#if not SECOND_ROBOT
    //! @def GOAL_SEMI_CIRCLE_RADIUS_CM @brief The defender's arc orbit around goal
    #define GOAL_SEMI_CIRCLE_RADIUS_CM 243 //243
    //! @def DEFENCE_SURGE_STR_VALUE @brief Min strength value that allows robot to 
    //!                                     surge (Defence)
    #define DEFENCE_SURGE_STR_VALUE 135
#else
    //! @def GOAL_SEMI_CIRCLE_RADIUS_CM @brief The defender's arc orbit around goal
    #define GOAL_SEMI_CIRCLE_RADIUS_CM 243
    //! @def DEFENCE_SURGE_STR_VALUE @brief Min strength value that allows robot to 
    //!                                     surge (Defence)
    #define DEFENCE_SURGE_STR_VALUE 85
#endif

// --[GOAL TRACKING Values]--
#if not SECOND_ROBOT
    //! @def GOAL_TRACKING_DIS_THRESH @brief Distance away from the goal that the 
    //!                               robot starts goal tracking with the orbit
    #define GOAL_TRACKING_DIS_THRESH 60
    //! @def GOAL_DIS_OFFSET @brief Offset value when calculating the goal distance
    //!                             on one side
    #define GOAL_DIS_OFFSET -101
    //! @def DEF_GOAL_Y_THRESH @brief Thresh to determine if the robot goes
    //                            for or back if it does not see goal (Defence)
    #define DEF_GOAL_Y_THRESH -40
#else
    //! @def GOAL_TRACKING_DIS_THRESH @brief Distance away from the goal that the 
    //!                               robot starts goal tracking with the orbit
    #define GOAL_TRACKING_DIS_THRESH 60
    //! @def GOAL_DIS_OFFSET @brief Offset value when calculating the goal distance
    //!                             on one side
    #define GOAL_DIS_OFFSET -101
    //! @def DEF_GOAL_Y_THRESH @brief Thresh to determine if the robot goes
    //                            for or back if it does not see goal (Defence)
    #define DEF_GOAL_Y_THRESH -40
#endif

// --[BATTERY TRACKING Values]--
#if not SECOND_ROBOT
    //! @def BAT_MOTOROFF_THRESH @brief Thresh to determine if motor switch off
    #define BAT_MOTOROFF_THRESH 0.5
#else
    //! @def BAT_MOTOROFF_THRESH @brief Thresh to determine if motor switch off
    #define BAT_MOTOROFF_THRESH 0.5
#endif
    //! @def BATTERY_CRITICAL @brief The battery level where the battery is critical
    #define BATTERY_CRITICAL 11.7

// --[PHYSICAL DEBUG TOGGLES]
    //! @def CORRECTION_TEST @brief Testing Correction Only if True
    #define CORRECTION_TEST false
    //! @def BALL_FOLLOW_TEST @brief Testing Corretion + Ball Follow if true
    #define BALL_FOLLOW_TEST false
    //! @def GOAL_TRACKING_TOGGLE @brief If the robot should goal track
    #define GOAL_TRACKING_TOGGLE false

// --[DEBUG TOGGLES]--
    //! @def DEBUG_MOTORS @brief Read what the motors are sending when true
    #define DEBUG_MOTORS false
    //! @def DEBUG_IMU @brief Prints various values for camera and BNO when true
    #define DEBUG_IMU_CAM false
    //! @def DEBUG_TSSP @brief Prints various values for tssp when true
    #define DEBUG_TSSP false
    //! @def DEBUG_TSSP_SENSOR_VAL @brief Prints tssp sensor values when true
    #define DEBUG_TSSP_SENSOR_VAL false
    //! @def DEBUG_READ_RAWVAL @brief Prints the raw analog value from bat
    #define BAT_READ_RAWVAL false
    //! @def BAT_READ_VOLTS @brief Reads the approx voltage value (2 d.p.) from bat
    #define BAT_READ_VOLTS false
    //! @def DEBUG_ROBOT_STATE @brief Prints what the robot is currently doing
    #define DEBUG_ROBOT_STATE false  

#endif