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
    #define targetGoal false
    //! @def SURGE_SPEED @brief Speed that is set for surging
    #define  SURGE_SPEED 125
    //! @def BASE_SPEED @brief Speed that is set for minimum
    #define  BASE_SPEED 125
    //! @def SECOND_ROBOT @brief Defines if the second or first robot is being tuned
    #define SECOND_ROBOT true
    //! @def COMPETITION_MODE @brief If the robot is or is not in competition
    #define COMPETITION_MODE false

// --[SURGE STATE Values]--
struct surgeState {
    bool surgeQ;
    uint32_t startMillis;
};

// --[PID Values]--
#if not SECOND_ROBOT
    #define KP_IMU 0.9
    #define KI_IMU 0.0
    #define KD_IMU 0.055

    #define KP_CAM_ATTACK 1.4
    #define KI_CAM_ATTACK 0.0
    #define KD_CAM_ATTACK 0.04125

    #define KP_CAM_DEFEND 0.65
    #define KI_CAM_DEFEND 0
    #define KD_CAM_DEFEND 0.03

    #define KP_DEFEND_VERT 40.0
    #define KI_DEFEND_VERT 0.0
    #define KD_DEFEND_VERT 0.0

    #define KP_DEFEND_HOZT 2.0
    #define KI_DEFEND_HOZT 0.0
    #define KD_DEFEND_HOZT 0.0

    #define KP_LINE_AVOID 40.0
    #define KI_LINE_AVOID 0.0
    #define KD_LINE_AVOID 0.0
#else
    #define KP_IMU 0.9
    #define KI_IMU 0.0
    #define KD_IMU 0.0275

    #define KP_CAM_ATTACK 0.815
    #define KI_CAM_ATTACK 0.0
    #define KD_CAM_ATTACK 0.020625

    #define KP_CAM_DEFEND 0.65
    #define KI_CAM_DEFEND 0
    #define KD_CAM_DEFEND 0.03

    #define KP_DEFEND_VERT 40.0
    #define KI_DEFEND_VERT 0.0
    #define KD_DEFEND_VERT 0.0

    #define KP_DEFEND_HOZT 2.0
    #define KI_DEFEND_HOZT 0.0
    #define KD_DEFEND_HOZT 0.1

    #define KP_LINE_AVOID 40.0
    #define KI_LINE_AVOID 0.0
    #define KD_LINE_AVOID 0.0

    #define KP_CENTERING 1.0
    #define KI_CENTERING 0.0
    #define KD_CENTERING 0.0
#endif

// --[ATTACK LOGIC Values]
#if not SECOND_ROBOT
    //! @def ORBIT_STRENGTH_RADIUS @brief The strength value that the robot switches
    // strats for orbit
    #define ORBIT_STRENGTH_RADIUS 85
    //! @def SURGE_STR_VALUE @brief Min Strength value that allows the robot to 
    //!                             surge (Attack)
    #define SURGE_STR_VALUE 104
    //! @def EXPO_MIN_VAL @brief Minimum value of the exponential orbit
    #define EXPO_MIN_VAL 60
#else
    //! @def ORBIT_STRENGTH_RADIUS @brief The strength value that the robot switches
    // strats for orbit
    #define ORBIT_STRENGTH_RADIUS 100
    //! @def SURGE_STR_VALUE @brief Min Strength value that allows the robot to 
    //!                             surge (Attack)
    #define SURGE_STR_VALUE 107
    //! @def EXPO_MIN_VAL @brief Minimum value of the exponential orbit
    #define EXPO_MIN_VAL 60
#endif

// --[GOAL TRACKING Values]--
#if not SECOND_ROBOT
    //! @def GOAL_TRACKING_DIS_THRESH @brief Distance away from the goal that the 
    //!                               robot starts goal tracking with the orbit
    #if targetGoal
        #define GOAL_TRACKING_DIS_THRESH 60
    #else
        #define GOAL_TRACKING_DIS_THRESH 20
    #endif
    //! @def GOAL_DIS_OFFSET @brief Offset value when calculating the goal distance
    //!                             on one side
    #define GOAL_DIS_OFFSET -101
    #if targetGoal
        //! @def DEF_GOAL_Y_THRESH @brief Thresh to determine if the robot goes
        //                            for or back if it does not see goal (Defence)
        #define DEF_GOAL_Y_THRESH -33
    #else
        //! @def DEF_GOAL_Y_THRESH @brief Thresh to determine if the robot goes
        //                            for or back if it does not see goal (Defence)
        #define DEF_GOAL_Y_THRESH -40
    #endif
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

// --[LIGHT SENSOR Values]--
#define LS_FLIP_THRESH 90
#define ATK_LINE_SP 0.2
#define DEF_LINE_SP 0.5
#define LINE_STRICT_AVOID 0.75
#define DEF_VERT_SP 1

// --[BATTERY TRACKING Values]--
#if not SECOND_ROBOT
    //! @def BAT_MOTOROFF_THRESH @brief Thresh to determine if motor switch off
    #define BAT_MOTOROFF_THRESH 0.5
#else
    //! @def BAT_MOTOROFF_THRESH @brief Thresh to determine if motor switch off
    #define BAT_MOTOROFF_THRESH 3
#endif
    //! @def BATTERY_CRITICAL @brief The battery level where the battery is critical
    #define BATTERY_CRITICAL 11.1
    //! @def BATTERY1_DIVIDER
    #define BATTERY1_DIVIDER 70.5

// --[PHYSICAL DEBUG TOGGLES]
    //! @def DEBUG_ROBOT @brief Testing space for individual items
    #define DEBUG_ROBOT false
    //! @def GOAL_TRACKING_TOGGLE @brief If the robot should goal track
    #define GOAL_TRACKING_TOGGLE true
    //! @def CAM_PACKET_SIZE @brief Size of the sent data accross camera
    #define CAM_PACKET_SIZE 6
    //! @def CAM_START_PACK_1 @brief Size of the sent data accross camera
    #define CAM_START_PACK_1 200
    //! @def CAM_START_PACK_2 @brief Size of the sent data accross camera
    #define CAM_START_PACK_2 122

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
    //! @def DEBUG_LS @brief Reads LS values
    #define DEBUG_LS false
    //! @def DEBUG_LS_SENSOR @brief Reads individual LS values
    #define DEBUG_LS_SENSOR false
    //! @def DEBUG_LINE_STATE @brief Reads individual LS values
    #define DEBUG_LINE_STATE false

    #define DEBUG false
#endif