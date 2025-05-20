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
PID defenderMovementHozt(PID_p_defender_movement_vert, PID_i_defender_movement_vert, \
                     PID_d_defender_movement_vert, SET_SPEED);
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
float goalHeading = 0;
//! @brief The amount the robot must correct to stay forward in terms of compass
float bnoHeading = 0;
//! @brief The amount of correction in terms of heading for the bno
float bnoCorrection = 0;
//! @brief The amount of correction in terms of heading for the camera (attack)
float cameraAttackCorrection = 0;
//! @brief The amount of correction in terms of heading for the camera (defend)
float cameraDefenceCorrection = 0;
//! @brief Angle to goal of the chosen goal that is being tracked.
float goal_angle = 0;
//! @brief Distance of goal to robot (horizontal - pixels)
float goal_dis = 0;
//! @brief Current Compass Value (rotation)
float rot = 0;
//! @brief The orbit values of the attacking function
float attackerMoveDirection = 0;
//! @brief The movement speed of the robot based on function (attack)
float attackerMoveSpeed = 0;
//! @brief The movement based on the veritcal PID of the robot (defend)
float verticalDefenderMovement = 0;
//! @brief The movement based on the horizontal PID of the robot (defend) 
float horizontalDefenderMovement = 0;
//! @brief Net Defender Movement Angle with Hozt and Vert calcs 
float netDefendMovementAngle = 0;
//! @brief Net Defender Speed with HOzt and Vert calc
float netDefendSpeed = 0;
//! @brief Current battery level of the robot (amps)
float batteryCurrentLevel = 0;
//! @brief Current battery level of the robot (volts)
float currentBatteryLevelVolts = 0;
//! @brief If the motor switch is turned on
bool motorOn = false;
//! @brief Current logic state of the robot
String robotState = "default";
//! @brief State that the correction is in
String correctionState = "default";

void setup() {
    Serial.begin(9600);
    tssp.init();
    ls.init();
    motors.init();
    bluetooth.init();
    batteryLevel.init();
    cam.init();
    compass.setExtCrystalUse(true);
    pinMode(LOGIC_PIN, INPUT);
    while(!compass.begin()) {
        Serial.println("bno ded ;(");
    }
}

void loop() {
// [Tssp Calculations]
    tssp.update();
    #if DEBUG_TSSP
        Serial.print(tssp.ballDir);
        Serial.print("\t");
        Serial.print(tssp.detectingBall);
        Serial.print("\t");
        Serial.println(tssp.ballStr);
    #endif

// [Logic Pin Calculations]
    // dirCalc.attack = digitalRead(LOGIC_PIN);

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
        goal_angle = cam.angle_to_goal_yellow;
    #endif
    // Complete floatMod values to ensure that the heading is not constantly 
    // changing when the robot faces the goal.
    goalHeading = (dirCalc.attack)?
                                        (floatMod(-1*goal_angle, 360) > 180 ? \
                                        floatMod(-1*goal_angle, 360) - 360 : \
                                        floatMod(-1*goal_angle, 360)) : \
                                        floatMod(-1*goal_angle, 360);
    // Regular correction using the BNO/IMU/Compass
    bnoHeading = (rot>180)?(rot-360):rot;
    // Calculate distance of the goals away from the robot (pixels)
    goal_dis = (sqrt(pow(abs(goal_x_val), 2) + pow(abs(goal_y_val), 2)))/10;
    // Hard coded offset value as the camera was reading different values on
    // each side.
    goal_dis = (goalHeading < 180)?(goal_dis - 10.1) : goal_dis;
    // Assign PID's variables
    bnoCorrection = -1*regularCorrection.update(bnoHeading, 0);
    cameraAttackCorrection = 1*goalAttackingCorrection.update(goalHeading, 0);
    cameraDefenceCorrection = -1*goalDefendingCorrection.update(goalHeading, \
                                                                180);
    // Heading logic to assign goal tracking or regular compass correct
    #if GOAL_TRACKING_TOGGLE
        if(goal_x_val == 0 || goal_y_val == 0) {
            // If not seeing goal --> Regular Correction
            correction = bnoCorrection;
            correctionState = "No data";
        } else {
            // If seeing goal
            if(dirCalc.attack) {
                // If Attacking
                if(tssp.ballDir <= 30 || tssp.ballDir >= 330) {
                    correction = cameraAttackCorrection;
                    correctionState = "Goal";
                } else {
                    correction = bnoCorrection;
                    correctionState = "Regular";
                }
            } else {
                // If Defending
                correction = cameraDefenceCorrection;
                correctionState = "Defence";
            }
        }
    #else
        correction = bnoCorrection;
    #endif
    #if DEBUG_IMU_CAM
        Serial.print(rot);
        Serial.print("\t");
        Serial.print(bnoHeading);
        Serial.print("\t");
        Serial.print(goalHeading);
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
        Serial.println(correction);
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
    attackerMoveSpeed = dirCalc.calcSpeed(tssp.ballStr)*SET_SPEED;
    verticalDefenderMovement = -defenderMovement.update(abs(goal_dis), \
                                                GOAL_SEMI_CIRCLE_RADIUS_CM);
    horizontalDefenderMovement = -defenderMovementHozt.update(\
                (tssp.ballDir > 180) ? (tssp.ballDir - 360) : tssp.ballDir, 0);
    netDefendMovementAngle = floatMod(atan2(horizontalDefenderMovement, \
                             verticalDefenderMovement)*RAD_TO_DEG, 360);
    netDefendSpeed = sqrt(pow(verticalDefenderMovement, 2) + \
                         pow(horizontalDefenderMovement, 2));

// [Bluetooth]
    bluetooth.update(batteryLevel.volts, tssp.ballDir, 0);

// [Moving the Robot Final Calculations and Logic]
    #if CORRECTION_TEST
        motors.run(0, 0, correction);
    #elif BALL_FOLLOW_TEST
        motors.run((tssp.detectingBall?attackerMoveSpeed:0), 
                  tssp.ballDir, 0); 
    #else
        if(ls.lineDirection != -1) {
            // If detecting line --> Line Avoidance
            robotState = "Line Avoidance";
            motors.run(attackerMoveSpeed, floatMod(ls.lineDirection + 180, 360),
                       0);
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
                        motors.run((tssp.detectingBall?attackerMoveSpeed:0),
                                attackerMoveDirection, correction);
                        robotState = "Attacker Logic - Orbit";
                    } else {
                        // If ball is far then ball follow
                        motors.run((tssp.detectingBall?attackerMoveSpeed:0), 
                                tssp.ballDir, correction);
                        robotState = "Attacker Logic - Ball Follow";
                    }
                }
            } else {
                // Defender Logic
                if(((tssp.ballDir >= 350 || tssp.ballDir <= 10) && \
                    (tssp.ballStr >= SURGE_STR_VALUE)) && \
                    (goal_angle <= 190 && goal_angle >= 170)) {
                    // If ball in capture and goal is behind robot --> surge
                        motors.run(SET_SPEED, tssp.ballDir, correction);
                        robotState = "Defender Logic - Surge";
                } else {
                    // If ball not in capture
                    motors.run(netDefendSpeed, netDefendMovementAngle, 
                                correction);
                    robotState = "Defender Logic - Reg Position";
                }
            }
        }
    #endif
    
    #if DEBUG_ROBOT_STATE
        Serial.print(robotState);
        Serial.print("\t");
        Serial.print(correctionState);
        Serial.print("\t Detecting Ball?: ");
        Serial.println(tssp.detectingBall);
    #endif

// [Manual Printing Space]
    Serial.print(goal_dis);
    Serial.print("\t");
    Serial.print(goalHeading);
    Serial.print("\t");
    Serial.print(verticalDefenderMovement);
    Serial.print("\t");
    Serial.print(horizontalDefenderMovement);
    Serial.print("\t");
    Serial.print(netDefendMovementAngle);
    Serial.print("\t");
    Serial.println(netDefendSpeed);
}