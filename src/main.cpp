// [Includes]
#include <Arduino.h>
#include <pins.h>
#include <common.h>
#include <config.h>
#include <drive_system.h>
#include <tssp_system.h>
#include <LSLB.h>
#include <DirCalc.h>
#include <PID.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <camera.h>
#include <batread.h>
#include <bluetooth.h>
#include <camera.h>
#include <Wire.h>

// [Instances]
Drive_system motors;
Bluetooth bluetooth;
Tssp_system tssp;
LSystem ls;
Camera cam;
DirectionCalc dirCalc;
bno::Adafruit_BNO055 compass = bno::Adafruit_BNO055(-1, 0x29, &Wire);
PID regularCorrection(PID_p_attack, PID_i_attack, PID_d_attack, PID_abs_max);
PID goalAttackingCorrection(PID_p_attack, PID_i_attack, PID_d_attack, PID_abs_max);
PID goalDefendingCorrection(PID_p_defend, PID_i_defend, PID_d_defend, PID_abs_max);
PID defenderMovement(PID_p_defender_movement, PID_i_defender_movement, \
                     PID_d_defender_movement, SET_SPEED);
sensors_event_t rotation;
BatRead batteryLevel;

// [Main Global Vars]
//! @brief Y value of the chosen goal that is being tracked.
uint8_t goal_y_val = 0;
//! @brief X value of the chosen goal that is being tracked.
uint8_t goal_x_val = 0;
//! @brief Amount needed to turn (degs) to ensure that you are goal tracking/
//!        staying forward depending on case.
float correction = 0;
//! @brief The amount the robot must correct to stay forward in terms of camera
float goalTrackingCorrection = 0;
//! @brief Angle to goal of the chosen goal that is being tracked.
float goal_angle = 0;
//! @brief Distance of goal to robot (horizontal - pixels)
float goal_dis = 0;
//! @brief The amount the robot must correct to stay forward in terms of compass
float regularTrackingCorrection = 0;
//! @brief Current Compass Value (rotation)
float rot = 0;
//! @brief The orbit values of the attacking function
float attackerMoveDirection = 0;
//! @brief The orbit values of the defending function
float defenderMoveDirection = 0;
//! @brief The movement speed of the robot based on function (attack)
float moveSpeed = 0;
//! @brief The movement speed of the robot bsaed on PID (defend)
float defenderMoveSpeed = 0;
//! @brief Current battery level of the robot (amps)
float batteryCurrentLevel = 0;
//! @brief Current battery level of the robot (volts)
float currentBatteryLevelVolts = 0;
//! @brief If the motor switch is turned on
bool motorOn = false;
//! @brief The heading the robot needs to switch to
float heading = 0;
//! @brief Current logic state of the robot
String robotState = "default";

void setup() {
    Serial.begin(9600);
    tssp.init();
    ls.init();
    motors.init();
    bluetooth.init();
    batteryLevel.init();
    cam.init();
    compass.setExtCrystalUse(true);
    while(!compass.begin()) {
        Serial.println("bno ded ;(");
    }
}

void loop() {
// [Correction / Goal Tracking Calculations]
    compass.getEvent(&rotation);
    rot = rotation.orientation.x;
    cam.read_camera();
    
    // Decide which goal is being tracked
    #if targetGoal
        goal_y_val = cam.goal_y_blue;
        goal_x_val = cam.goal_x_blue;
        goal_angle = cam.angle_to_goal_blue;
    #else
        goal_y_val = cam.goal_y_yellow;
        goal_x_val = cam.goal_x_yellow;
        goal_angle = cam.angle_to_goal_blue;
    #endif
    // Complete floatMod values to ensure that the heading is not constantly 
    // changing when the robot faces the goal.
    goalTrackingCorrection = (dirCalc.attack)?
                                        (floatMod(-1*goal_angle, 360) > 180 ? \
                                        floatMod(-1*goal_angle, 360) - 360 : \
                                        floatMod(-1*goal_angle, 360)) : \
                                        floatMod(-1*goal_angle, 360);
    // Regular correction using the BNO/IMU/Compass
    regularTrackingCorrection = (rot>180)?(rot-360):rot;
    // Heading logic to assign goal tracking or regular compass correct
    #if GOAL_TRACKING_TOGGLE
        if(goal_x_val == 0 || goal_y_val == 0) {
            // If not seeing goal --> Regular Correction
            correction = -1*regularCorrection.update(regularTrackingCorrection, 0);
        } else {
            // If seeing goal
            if(dirCalc.attack) {
                // If Attacking
                correction = -1*goalAttackingCorrection.update(goalTrackingCorrection, 0);
            } else {
                // If Defending
                correction = -1*goalDefendingCorrection.update(goalTrackingCorrection, 180);
            }
        }
    #else
        correction = -1*regularCorrection.update(regularTrackingCorrection, 0);
    #endif
    // Assigns final correction value based on assigned heading in above logic
    // and if you are attacking/defending
    // if(!regularCorrection) {
    //     correction = (dirCalc.attack)?(-1*goalAttackingCorrection.update(heading, 0)): \
    //                  (-1*(goalDefendingCorrection.update(heading, 180)));
    // } else {
    //     correction = -1*goalAttackingCorrection.update(heading, 0);
    // }
    
    // Calculate distance of the goals away from the robot (pixels)
    goal_dis = (sqrt(pow(abs(goal_x_val), 2) + pow(abs(goal_y_val), 2)))/10;
    goal_dis = (goalTrackingCorrection < 180)?(goal_dis - 10.1) : goal_dis;

    #if DEBUG_IMU_CAM
        Serial.print(rot);
        Serial.print("\t");
        Serial.print(regularTrackingCorrection);
        Serial.print("\t");
        Serial.print(goalTrackingCorrection);
        Serial.print("\t");
        Serial.print(cam.goal_x_blue);
        Serial.print("\t");
        Serial.print(cam.goal_y_blue);
        Serial.print("\t"); 
        Serial.print(cam.angle_to_goal_blue);
        Serial.print("\t");
        Serial.print(cam.goal_x_yellow);
        Serial.print("\t");
        Serial.print(cam.goal_y_yellow);
        Serial.print("\t"); 
        Serial.print(cam.angle_to_goal_yellow);
        Serial.print("\t"); 
        Serial.print(heading);
        Serial.print("\t");
        Serial.println(correction);
    #endif


// [Tssp Calculations]
    tssp.update();
    #if DEBUG_TSSP
        Serial.print(tssp.ballDir);
        Serial.print("\t");
        Serial.print(tssp.detectingBall);
        Serial.print("\t");
        Serial.println(tssp.ballStr);
    #endif


// [Battery Level Calculations]
    batteryLevel.read();
    batteryLevel.toggleLED();

    #if BAT_READ_VOLTS
        if(batteryLevel.motorOn) {
            Serial.print("MOTOR ON");
        } else {
            Serial.print("MOTOR OFF");
        }
        Serial.print("\t");
        Serial.println(batteryLevel.volts);
    #endif


// [Strategy and Movement Calculation]
    attackerMoveDirection = dirCalc.exponentialOrbit(tssp.ballDir, 
                                                    tssp.ballStr);
    defenderMoveDirection = dirCalc.defenderMovement(goal_angle, goal_dis, 
                                                     tssp.ballDir, tssp.ballStr);     
    moveSpeed = dirCalc.calcSpeed(tssp.ballStr)*SET_SPEED;
    defenderMoveSpeed = defenderMovement.update(goal_dis, \
                                                GOAL_SEMI_CIRCLE_RADIUS_CM);
// [Bluetooth]
    bluetooth.update(batteryLevel.volts, tssp.ballDir, 0);

// [Moving the Robot Final Calculations and Logic]
    #if CORRECTION_TEST
        motors.run(0, 0, correction);
    #elif BALL_FOLLOW_TEST
        motors.run((tssp.detectingBall?moveSpeed:0), 
                  tssp.ballDir, 0); 
    #else
        if(ls.lineDirection != -1) {
            // If detecting line --> Line Avoidance
            robotState = "Line Avoidance";
            motors.run(moveSpeed, floatMod(ls.lineDirection + 180, 360), 0);
        } else {
            if(dirCalc.attack) {
                // If robot is attacking --> Attacker Logic
                if((tssp.ballDir >= 350 || tssp.ballDir <= 10) && \
                   tssp.ballStr >= SURGE_STR_VALUE) {
                    // If ball is generally straight and in capture --> surge
                    motors.run((tssp.detectingBall?SET_SPEED:0), tssp.ballDir, 
                                correction);
                    robotState = "Attacker Logic - Surge";
                } else {
                    if(tssp.ballStr > ORBIT_STRENGTH_RADIUS) {
                        // If ball is close then orbit
                        motors.run((tssp.detectingBall?moveSpeed:0),
                                attackerMoveDirection, correction);
                        robotState = "Attacker Logic - Orbit";
                    } else {
                        // If ball is far then ball follow
                        motors.run((tssp.detectingBall?moveSpeed:0), 
                                tssp.ballDir, correction);
                        robotState = "Attacker Logic - Ball Follow";
                    }
                }
            } else {
                // Defender Logic
                #if CORRECTION_TEST
                    motors.run(0, 0, correction);
                #else
                    if(((tssp.ballDir >= 350 || tssp.ballDir <= 10) && \
                        (tssp.ballStr >= SURGE_STR_VALUE))) {
                        // ((tssp.ballDir >= 350 || tssp.ballDir <= 10) && \
                        // (tssp.ballStr >= SURGE_STR_VALUE)) && \
                        // (goal_angle <= 10 && goal_angle >= -10)
                        // If ball in capture and infront of goal then surge.
                            motors.run(SET_SPEED, tssp.ballDir, correction);
                            robotState = "Defender Logic - Surge";
                    } else {
                        // If ball not in capture
                        if(defenderMoveDirection != -1) {
                            // If defender not on target line
                            motors.run(35, defenderMoveDirection, 
                                       correction);
                            robotState = "Defender Logic - Moving towards line";
                        } else {
                            // If defender on target line.
                            motors.run(0, 0, correction);
                            robotState = "Defender Logic - In Set Position";
                        }
                    }
                #endif
            }
        }
    #endif

    Serial.print(defenderMoveDirection);
    Serial.print("\t");
    Serial.print(goalTrackingCorrection);
    Serial.print("\t");
    Serial.println(goal_dis);

    #if DEBUG_ROBOT_STATE
        Serial.print(robotState);
        Serial.print("\t Detecting Ball?: ");
        Serial.println(tssp.detectingBall);
    #endif
}