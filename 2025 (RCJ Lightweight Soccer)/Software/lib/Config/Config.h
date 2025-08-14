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
    //! @def SURGE_SPEED @brief Speed that is set for surging
    #define  SURGE_SPEED 150
    //! @def BASE_SPEED @brief Speed that is set for minimum
    #define  BASE_SPEED 80
    //! @def SECOND_ROBOT @brief Defines if the second or first robot is being 
    //!                          tuned.
    #define SECOND_ROBOT false
    //! @def COMPETITION_MODE @brief If the robot is in international comp
    #define COMPETITION_MODE false
    //! @def GOAL_TRACKING_TOGGLE @brief If the robot should goal track
    #define GOAL_TRACKING_TOGGLE true

// --[PID Values]--
#if not SECOND_ROBOT
    //! @def KP_IMU @brief Proportional value for the IMU PID
    #define KP_IMU 1.0 //0.6
    //! @def KI_IMU @brief Intergral value for the IMU PID
    #define KI_IMU 0.0
    //! @def KD_IMU @brief Derivative value for the IMU PID
    #define KD_IMU 0.08 //0.21 0.67

    //! @def KP_CAM_ATTACK @brief Proportional value for the Goal Track Atk PID
    #define KP_CAM_ATTACK 1.0
    //! @def KI_CAM_ATTACK @brief Intergral value for the Goal Track Atk PID
    #define KI_CAM_ATTACK 0.0
    //! @def KD_CAM_ATTACK @brief Derivative value for the Goal Track Atk PID
    #define KD_CAM_ATTACK 0.0

    //! @def KP_CAM_DEFEND @brief Proportional value for the Goal Track Def PID
    #define KP_CAM_DEFEND 1.2
    //! @def KI_CAM_DEFEND @brief Intergral value for the Goal Track Def PID
    #define KI_CAM_DEFEND 0.0
    //! @def KD_CAM_DEFEND @brief Derivative value for the Goal Track Def PID
    #define KD_CAM_DEFEND 0.05

 //! @def KP_DEFEND_VERT @brief Proportional value for the Def Vert Goal Pos PID
    #define KP_DEFEND_VERT 6.0
 //! @def KI_DEFEND_VERT @brief Intergral value for the Def Vert Goal Pos PID
    #define KI_DEFEND_VERT 0.0
 //! @def KD_DEFEND_VERT @brief Derivative value for the Def Vert Goal Pos PID
    #define KD_DEFEND_VERT 0.0

//! @def KP_DEFEND_HOZT @brief Proportional value for the Def Hozt Goal Pos PID
    #define KP_DEFEND_HOZT 10.0
//! @def KI_DEFEND_HOZT @brief Intergral value for the Def Hozt Goal Pos PID
    #define KI_DEFEND_HOZT 0.0
//! @def KD_DEFEND_HOZT @brief Derivative value for the Def Hozt Goal Pos PID
    #define KD_DEFEND_HOZT 0.0

//! @def KP_LINE_AVOID @brief Proportional value for the line avoid PID
    #define KP_LINE_AVOID 200.0
//! @def KI_LINE_AVOID @brief Intergral value for the line avoid PID
    #define KI_LINE_AVOID 0.0
//! @def KD_LINE_AVOID @brief Derivative value for the line avoid PID
    #define KD_LINE_AVOID 0.0

//! @def KP_CENTERING @brief Proportional value for the field centering PID
    #define KP_CENTERING 2.0
//! @def KI_CENTERING @brief Intergral value for the field centering PID
    #define KI_CENTERING 0.0
//! @def KD_CENTERING @brief Derivative value for the field centering PID
    #define KD_CENTERING 0.0
#else
    //! @def KP_IMU @brief Proportional value for the IMU PID
    #define KP_IMU 1.0 //0.6
    //! @def KI_IMU @brief Intergral value for the IMU PID
    #define KI_IMU 0.0
    //! @def KD_IMU @brief Derivative value for the IMU PID
    #define KD_IMU 0.045 ///0.21 0.67

    //! @def KP_CAM_ATTACK @brief Proportional value for the Goal Track Atk PID
    #define KP_CAM_ATTACK 0.815
    //! @def KI_CAM_ATTACK @brief Intergral value for the Goal Track Atk PID
    #define KI_CAM_ATTACK 0.0
    //! @def KD_CAM_ATTACK @brief Derivative value for the Goal Track Atk PID
    #define KD_CAM_ATTACK 0.0

    //! @def KP_CAM_DEFEND @brief Proportional value for the Goal Track Def PID
    #define KP_CAM_DEFEND 0.05
    //! @def KI_CAM_DEFEND @brief Intergral value for the Goal Track Def PID
    #define KI_CAM_DEFEND 0.0
    //! @def KD_CAM_DEFEND @brief Derivative value for the Goal Track Def PID
    #define KD_CAM_DEFEND 1.0

 //! @def KP_DEFEND_VERT @brief Proportional value for the Def Vert Goal Pos PID
    #define KP_DEFEND_VERT 120.0
 //! @def KI_DEFEND_VERT @brief Intergral value for the Def Vert Goal Pos PID
    #define KI_DEFEND_VERT 0.0
 //! @def KD_DEFEND_VERT @brief Derivative value for the Def Vert Goal Pos PID
    #define KD_DEFEND_VERT 0.0

//! @def KP_DEFEND_HOZT @brief Proportional value for the Def Hozt Goal Pos PID
    #define KP_DEFEND_HOZT 3.0
//! @def KI_DEFEND_HOZT @brief Intergral value for the Def Hozt Goal Pos PID
    #define KI_DEFEND_HOZT 0.0
//! @def KD_DEFEND_HOZT @brief Derivative value for the Def Hozt Goal Pos PID
    #define KD_DEFEND_HOZT 0.0

//! @def KP_LINE_AVOID @brief Proportional value for the line avoid PID
    #define KP_LINE_AVOID 200.0
//! @def KI_LINE_AVOID @brief Intergral value for the line avoid PID
    #define KI_LINE_AVOID 0.0
//! @def KD_LINE_AVOID @brief Derivative value for the line avoid PID
    #define KD_LINE_AVOID 0.0

//! @def KP_CENTERING @brief Proportional value for the field centering PID
    #define KP_CENTERING 20.0
//! @def KI_CENTERING @brief Intergral value for the field centering PID
    #define KI_CENTERING 0.0
//! @def KD_CENTERING @brief Derivative value for the field centering PID
    #define KD_CENTERING 0.0
#endif

// --[LOGIC Values]
#if not SECOND_ROBOT
    //! @def ORBIT_STRENGTH_RADIUS @brief The strength value that the robot 
    //!                                   switches
    // strats for orbit
    #define ORBIT_STRENGTH_RADIUS 60
    //! @def DEFEND_SURGE @brief The ball strength at which the defender
    //!                          switches to attacker.
    #define DEFEND_SURGE 110
    //! @def DEFEND_NOLINE_MSPD @brief Movement at which the defender moves at
    //!                                when not seeing the line.
    #define DEFEND_NOLINE_MSPD 50
    //! @def ATK_LINE_SP @brief Target line state of the attacker robots line 
    //!                         avoid PID.
    #define ATK_LINE_SP 0.4
    //! @def DEF_VERT_SP @brief The set line state which the defender follows.
    #define DEF_VERT_SP 0.5
#else
    //! @def ORBIT_STRENGTH_RADIUS @brief The strength value that the robot 
    //!                                   switches
    // strats for orbit
    #define ORBIT_STRENGTH_RADIUS 60
    //! @def DEFEND_SURGE @brief The ball strength at which the defender
    //!                          switches to attacker.
    #define DEFEND_SURGE 45
    //! @def DEFEND_NOLINE_MSPD @brief Movement at which the defender moves at
    //!                                when not seeing the line.
    #define DEFEND_NOLINE_MSPD 50
    //! @def ATK_LINE_SP @brief Target line state of the attacker robots line 
    //!                         avoid PID.
    #define ATK_LINE_SP 0.1
    //! @def DEF_VERT_SP @brief The set line state which the defender follows.
    #define DEF_VERT_SP 0.5
#endif

// --[LIGHT SENSOR Values]--
    //! @def LS_CLB_THRESH @brief Threshold to determine if a sensor is
    //!                           detecting white.
    #define LS_CLB_THRESH 200 // 50
    //! @def LS_FLIP_THRESH @brief Threshold to determine when the robot
    //!                            flips from state 1 to 2, or 2 to 1.
    #define LS_FLIP_THRESH 90

// --[BLUETOOTH Values]
    //! @def BT_SERIAL @brief Serial used to transfer data from the
    //!                              bluetooth modules
    #define BT_SERIAL Serial1
    //! @def BT_BAUD @brief Bluetooth communication baud rate.
    #define BT_BAUD 9600
    //! @def BT_PACKET_SIZE @brief Size of data being sent over.
    #define BT_PACKET_SIZE 6
    //! @def BT_START_BYTE @brief Byte start identifier.
    #define BT_START_BYTE 255
    //! @def BT_NO_DATA @brief If module has no data output.
    #define BT_NO_DATA 255`

// --[CAMERA Values]--
    //! @def cameraSerial @brief Serial that the camera sends data over to
    #define cameraSerial Serial8
    //! @def CAM_PACKET_SIZE @brief Size of the sent data accross camera
    #define CAM_PACKET_SIZE 6
    //! @def CAM_START_PACK_1 @brief First start byte of camera.
    #define CAM_START_PACK_1 200
    //! @def CAM_START_PACK_2 @brief Second start byte of camera.
    #define CAM_START_PACK_2 122

// --[BATTERY TRACKING Values]--
    //! @def BATTERY_CRITICAL @brief The battery level where the battery is 
    //!                              critical.
    #define BATTERY_CRITICAL 11.0
    //! @def BATTERY1_DIVIDER @brief The divider of the analogue value to 
    //!                              achieve a battery level in volts.
    #define BATTERY1_DIVIDER 71

// --[DEBUG TOGGLES]--
    //! @def DEBUG_BLUETOOTH @brief Allows printing and debugging of bluetooth
    //!                             values.
    #define DEBUG_BLUETOOTH false
    //! @def DEBUG_CAMERA @brief Allows printing and debugging of camera values.
    #define DEBUG_CAMERA false
    //! @def DEBUG_MOTORS @brief Allows printing and debugging of sent motor
    //!                          values.
    #define DEBUG_MOTORS false
    //! @def DEBUG_LS_VALS @brief Allows printing and debugging of individual
    //!                           sensor values for testing.
    #define DEBUG_LS_VALS false
    //! @def DEBUG_LS_TRIG @brief Allows printing and debugging of individual
    //!                           triggered values for testing.
    #define DEBUG_LS_TRIG false
    //! @def DEBUG_LS_CALCS @brief Allows printing and debugging of the calcs
    //!                            involved in the LS library.
    #define DEBUG_LS_CALCS false
    //! @def DEBUG_LS @brief Allows printing and debugging of the light sensor
    //!                      final values for testing.
    #define DEBUG_LS false
    //! @def DEBUG_TSSP_SENSOR_VAL @brief Allows printing of tssp values for
    //!                                   testing.
    #define DEBUG_TSSP_VALS  false
    //! @def DEBUG_TSSP @brief Allows printing and debugging of the tssp sensor
    //!                        final values for testing.
    #define DEBUG_TSSP false
    //! @def DEBUG_VD @brief Allows printing and debuggin of the voltage divider
    //!                      for testing and comparison.
    #define DEBUG_VD false
#endif