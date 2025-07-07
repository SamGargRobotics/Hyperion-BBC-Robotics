// [Includes]
#include <Arduino.h>
#include <pins.h>
#include <common.h>
#include <config.h>
#include <drive_system.h>
#include <tssp_system.h>
#include <light_system.h>
#include <DirCalc.h>
#include <PID.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <camera.h>
#include <batread.h>
#include <bluetooth.h>
#include <camera.h>
#include <Wire.h>
#include <Timer.h>

// [Instances]
Drive_system motors;
Bluetooth bluetooth;
Tssp_system tssp;
LSystem ls;
Camera cam;
DirectionCalc dirCalc;
bno::Adafruit_BNO055 compass = bno::Adafruit_BNO055(-1, 0x29, &Wire);
sensors_event_t rotation;
BatRead batteryLevel;
Timer batteryTimer(5000000);
PID regularCorrection(PID_p_attack, PID_i_attack, PID_d_attack, PID_abs_max);
PID goalAttackingCorrection(PID_p_attack_goal, PID_i_attack, PID_d_attack*3/4, \
                            PID_abs_max);
PID goalDefendingCorrection(PID_p_defend, PID_i_defend, PID_d_defend, \
                            PID_abs_max);
PID defenderMovementVert(PID_p_defender_movement_vert, \
                         PID_i_defender_movement_vert, \
                         PID_d_defender_movement_vert, PID_abs_max);
PID defenderMovementHozt(PID_p_defender_movement_hozt, \
                         PID_i_defender_movement_hozt, \
                         PID_d_defender_movement_hozt, PID_abs_max);
surgeState surgestates;

// [Main Global Vars]
//! @brief Y value of the chosen goal that is being tracked.
uint8_t goal_y_val = 0;
//! @brief X value of the chosen goal that is being tracked.
uint8_t goal_x_val = 0;
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
//! @brief Goal pixel size.
float goal_pixel = 0;
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
//! @brief Move Angle in relevancy to the ls.lineDirection
float lsMoveAngle = -1;
//! @brief Current logic state of the robot
String robotState = "default";
//! @brief State that the correction is in
String correctionState = "default";

bool chosenGoal = targetGoal;

void setup() {
    Serial.begin(9600);
    tssp.init();
    ls.init();
    motors.init();
    bluetooth.init();
    batteryLevel.init();
    batteryTimer.resetTime();
    cam.init();
    compass.setExtCrystalUse(true);
    pinMode(LOGIC_PIN, INPUT);
    while(!compass.begin()) {
        Serial.println("bno ded ;(");
    }
}

void loop() {
// [Logic Pin Calculations]
    chosenGoal = digitalRead(LOGIC_PIN);

// [Tssp Calculations]
    tssp.update();
    #if DEBUG_TSSP
        Serial.print(tssp.ballDir);
        Serial.print("\t");
        Serial.print(tssp.detectingBall);
        Serial.print("\t");
        Serial.println(tssp.ballStr);
    #endif

// [Bluetooth]
    dirCalc.attack = bluetooth.update(dirCalc.attack, tssp.ballDir, tssp.ballStr);
    dirCalc.attack = 1;
    
// [Correction / Goal Tracking Calculations]
    compass.getEvent(&rotation);
    rot = rotation.orientation.x;
    cam.read_camera();
    
    // Decide which goal is being tracked
    if(chosenGoal) {
        goal_y_val = (dirCalc.attack)? cam.goal_y_blue : cam.goal_y_yellow;
        goal_x_val = (dirCalc.attack)? cam.goal_x_blue : cam.goal_x_yellow;
        goal_angle = (dirCalc.attack)? cam.angle_to_goal_blue : \
                                       cam.angle_to_goal_yellow;
        goal_pixel = cam.distBlue;
    } else {
        goal_y_val = (dirCalc.attack)? cam.goal_y_yellow : cam.goal_y_blue;
        goal_x_val = (dirCalc.attack)? cam.goal_x_yellow : cam.goal_x_blue;
        goal_angle = (dirCalc.attack)? cam.angle_to_goal_yellow : \
                                       cam.angle_to_goal_blue;
        goal_pixel = cam.distYel;
    }
    // Complete floatMod values to ensure that the heading is not constantly 
    // changing when the robot faces the goal.
    goalHeading = (dirCalc.attack)? (floatMod(-goal_angle, 360) > 180 ? \
                                    floatMod(-goal_angle, 360) - 360 : \
                                    floatMod(-goal_angle, 360)) : \
                                    floatMod(-goal_angle, 360);
    // Regular correction using the BNO/IMU/Compass
    bnoHeading = (rot>180)?(rot-360):rot;
    // Assign PID's variables
    bnoCorrection = -regularCorrection.update(bnoHeading, 0);
    cameraAttackCorrection = -goalAttackingCorrection.update(goalHeading, 0);
    cameraDefenceCorrection = -goalDefendingCorrection.update(goalHeading, 180);
    // Heading logic to assign goal tracking or regular compass correct
    cameraAttackCorrection = ((goal_y_val == 0 || goal_x_val == 0) || \
    !((tssp.ballDir <= 110 || tssp.ballDir >= 290))) ? \
                              bnoCorrection:cameraAttackCorrection;
    cameraDefenceCorrection = (goal_y_val == 0 || goal_x_val == 0)? \
                              bnoCorrection:cameraDefenceCorrection;
    // Calculate distance of the goals away from the robot (pixels)
    goal_dis = (sqrt(pow(abs(goal_x_val), 2) + pow(abs(goal_y_val), 2)));
    // // Hard coded offset value as the camera was reading different values on
    // // each side.
    goal_dis = (goalHeading < 180)?(goal_dis + GOAL_DIS_OFFSET) : goal_dis;
    #if not GOAL_TRACKING_TOGGLE
        cameraAttackCorrection = bnoCorrection;
        cameraDefenceCorrection = bnoCorrection;
    #endif
    #if DEBUG_IMU_CAM
        Serial.print(rot);
        Serial.print("\t");
        Serial.print(bnoHeading);
        Serial.print("\t");
        Serial.print(bnoCorrection);
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
        Serial.println(goal_dis);
    #endif

// [Battery Level Calculations]
    batteryLevel.read();

    #if BAT_READ_VOLTS
        if(batteryLevel.motorOn) {
            Serial.print("MOTOR ON");
        } else {
            Serial.print("MOTOR OFF");
        }
        Serial.print("\t");
        Serial.println(batteryLevel.volts);
    #endif

// [Light Sensors]
    ls.calculateLineDirection(rot);
    lsMoveAngle = (ls.lineDir == -1)? -1 : floatMod((ls.lineDir - rot) + 180, 360);
    for(int i = 0; i < NUM_LS; i++) {
        Serial.print(ls.readOne(i));
        Serial.print(" ");
    }
    Serial.println();
    // Serial.println(ls.relativeDefenderMovement);
    #if DEBUG_LS
        Serial.print("ClusterAmt: ");
        Serial.print(ls.clusterAmount);
        Serial.print("\t"); 
        Serial.print("LineState: ");
        Serial.print(ls.lineState);
        Serial.print("\t");
        Serial.print("Move: ");
        Serial.print(lsMoveAngle);
        Serial.print("\t");
        Serial.print("Line: ");
        Serial.println(lineAngle);
    #endif

// [Strategy and Movement Calculation]
    attackerMoveDirection = dirCalc.exponentialOrbit(tssp.ballDir);  
    attackerMoveSpeed = dirCalc.calcSpeed(tssp.ballStr, tssp.ballDir)*SET_SPEED;
    attackerMoveSpeed = max(50.0f, ((goal_angle >= 5 || goal_angle <= -15) || (goal_x_val == 0 && goal_y_val == 0))? attackerMoveSpeed/4 : attackerMoveSpeed);
    float defenderVertHeading = (tssp.detectingBall) ? ((tssp.ballDir > 180) ? (tssp.ballDir - 360) : tssp.ballDir) : -bnoHeading;
    verticalDefenderMovement = -defenderMovementVert.update(abs(goal_dis), \
                                                GOAL_SEMI_CIRCLE_RADIUS_CM);
    horizontalDefenderMovement = -defenderMovementHozt.update(defenderVertHeading, 0);
    netDefendMovementAngle = floatMod(atan2(horizontalDefenderMovement, \
                             verticalDefenderMovement)*RAD_TO_DEG, 360);
    netDefendSpeed = sqrt(pow(verticalDefenderMovement, 2) + \
                         pow(horizontalDefenderMovement, 2));
    if ((tssp.ballDir >= 350 || tssp.ballDir <= 10) && \
        (tssp.ballStr >= DEFENCE_SURGE_STR_VALUE) && !surgestates.surgeQ) {
            surgestates.surgeQ = true;
            surgestates.startMillis = micros();
        }
    if((tssp.ballStr <= 60) || (surgestates.startMillis+5000000) <= micros() || \
        ((tssp.ballDir >= 10 && tssp.ballDir <= 350) || goal_dis <= 225)) {
        surgestates.surgeQ = false;
    }

// [Moving the Robot Final Calculations and Logic]
    float _moveSpeed = 0;
    float _moveAngle = 0;
    float _moveRotation = 0;
    #if DEBUG_ROBOT
        motors.run(0, 0, bnoCorrection);
    #else
        if(dirCalc.attack) {
            // If robot is attacking --> Attacker Logic
            if((tssp.ballDir >= 347.5 || tssp.ballDir <= 12.5) && \
                tssp.ballStr >= SURGE_STR_VALUE) {
                // If ball is generally straight and in capture --> surge
                _moveSpeed = tssp.detectingBall?SET_SPEED:0;
                _moveAngle = goal_angle;
                _moveRotation = cameraAttackCorrection;
                _moveRotation += random(-10,10);
                robotState = "Attacker Logic - Surge";
            } else {
                if(tssp.ballStr > ORBIT_STRENGTH_RADIUS) {
                    // If ball is close then orbit
                    _moveSpeed = tssp.detectingBall?attackerMoveSpeed:0;
                    _moveAngle = attackerMoveDirection;
                    _moveRotation = cameraAttackCorrection;
                    correctionState = "Regular";
                    robotState = "Attacker Logic - Orbit";
                } else {
                    // If ball is far then ball follow
                    _moveSpeed = tssp.detectingBall?attackerMoveSpeed:0;
                    _moveAngle = tssp.ballDir;
                    _moveRotation = cameraAttackCorrection;
                    robotState = "Attacker Logic - Ball Follow";
                }
            }
        } else {
            // Defender Logic
            if(surgestates.surgeQ) {
                _moveSpeed = SET_SPEED;
                _moveAngle = tssp.ballDir;
                _moveRotation = cameraAttackCorrection;
                correctionState = "Goal";
                robotState = "Defender Logic - Surge";
            } else {
                // If ball not in capture
                if(goal_x_val != 0 && goal_y_val != 0) {
                    // If can see goal
                    if(tssp.ballDir >= 110 && tssp.ballDir <= 290) {
                        // If ball is in threshold robot --> orbit
                        _moveSpeed = attackerMoveSpeed;
                        _moveAngle = attackerMoveDirection;
                        _moveRotation = bnoCorrection;
                        correctionState = "Regular";
                        robotState = "Defender Logic - Ball Behind";
                    } else {
                        _moveSpeed = netDefendSpeed;
                        _moveAngle = netDefendMovementAngle;
                        _moveRotation = cameraDefenceCorrection;
                        robotState = "Defender Logic - Regular";
                    }
                } else {
                    // If cannot see goal --> Forward or Backward depending
                    // on last seen goal_y_val
                    _moveSpeed = attackerMoveSpeed;
                    _moveAngle = (cam.previousVals[0] <= DEF_GOAL_Y_THRESH)?180:tssp.ballDir;
                    _moveRotation = bnoCorrection;
                    correctionState = "Regular";
                    robotState = "Defender Logic - Cannot see goal";
                }
            }
        }
        if(ls.lineDir != -1) {
            // If detecting line --> Line Avoidance
            _moveAngle = lsMoveAngle;
            _moveSpeed = SET_SPEED;
            robotState = "Line Avoidance";
        }
        //TEMP
        motors.run(_moveSpeed, _moveAngle, _moveRotation);
    #endif
    #if DEBUG_ROBOT_STATE
        Serial.print(robotState);
        Serial.print("\t");
        Serial.print(correctionState);
        Serial.print("\t Detecting Ball?: ");
        Serial.println(tssp.detectingBall);
    #endif

    // if battery level is less than battery critical level, spin in spot
    #if not COMPETITION_MODE
        if(batteryLevel.volts <= BATTERY_CRITICAL) {
            if(batteryTimer.timeHasPassedNoUpdate()) {
                motors.run(0,0,20);
            }
        } else {
            batteryTimer.resetTime();
        }
    #endif

// [Manual Printing Space]
// Serial.println
}