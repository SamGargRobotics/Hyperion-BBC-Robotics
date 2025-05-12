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
PID compass_correct(PID_p, PID_i, PID_d);
sensors_event_t rotation;
BatRead batteryLevel;

// [Main Global Vars]
//! @brief Amount needed to turn (degs) to ensure that you are goal tracking/
//!        staying forward depending on case.
float correction = 0;
//! @brief The amount the robot must correct to stay forward in terms of camera
float goalTrackingCorrection = 0;
//! @brief Goal tracking correction for blue goal
float blueGoalTarget = 0;
//! @brief Goal tracking correction for yellow goal
float yellowGoalTarget = 0;
//! @brief Y value of the chosen goal that is being tracked.
uint8_t goal_y_val = 0;
//! @brief X value of the chosen goal that is being tracked.
uint8_t goal_x_val = 0;
//! @brief The amount the robot must correct to stay forward in terms of compass
float regularTrackingCorrection = 0;
//! @brief Current Compass Value
float rot = 0;
//! @brief The orbit values of the attacking function
float attackerMoveDirection = 0;
//! @brief The orbit values of the defending function
float defenderMoveDirection = 0;
//! @brief The movement speed of the robot based on function
float moveSpeed = 0;
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
    compass.setExtCrystalUse(true);
    cam.init();
    while(!compass.begin()) {
        Serial.println("bno ded ;(");
    }
}

void loop() {
// [Correction / Goal Tracking Calculations]
    compass.getEvent(&rotation);
    rot = rotation.orientation.x;
    cam.read_camera();

    #if attackingGoal
        goal_y_val = cam.goal_y_blue;
        goal_x_val = cam.goal_x_blue;
    #else
        goal_y_val = cam.goal_y_yellow;
        goal_x_val = cam.goal_x_yellow;
    #endif
    blueGoalTarget = floatMod(-1*cam.angle_to_goal_blue, 360) > 180 ? \
                     floatMod(-1*cam.angle_to_goal_blue, 360) - 360 : \
                     floatMod(-1*cam.angle_to_goal_blue, 360);
    yellowGoalTarget = floatMod(-1*cam.angle_to_goal_yellow, 360) > 180 ? \
                       floatMod(-1*cam.angle_to_goal_yellow, 360) - 360 : \
                       floatMod(-1*cam.angle_to_goal_yellow, 360);
    goalTrackingCorrection = (attackingGoal) ? blueGoalTarget:yellowGoalTarget;
    regularTrackingCorrection = (rot>180)?(rot-360):rot;
    #if GOAL_TRACKING_TOGGLE
        if(goal_x_val == 0 && goal_y_val == 0) {
            heading = regularTrackingCorrection;
        } else {
            if(goal_y_val <= GOAL_TRACKING_DIS_THRESH) {
                heading = goalTrackingCorrection;
            } else {
                heading = regularTrackingCorrection;
            }
        }
    #else
        heading = regularTrackingCorrection;
    #endif

    correction = -1*compass_correct.update(heading, 0);

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
    // dirCalc.attack = dirCalc.calculateStrategy(
    //                                         bluetooth.otherRobotBallLocation[1],
    //                                         tssp.ballDir);
    attackerMoveDirection = dirCalc.exponentialOrbit(tssp.ballDir, 
                                                    tssp.ballStr);
    defenderMoveDirection = dirCalc.defenderMovement(0, 0, 
                                                    tssp.ballDir);                      
    moveSpeed = dirCalc.calcSpeed(tssp.ballStr)*SET_SPEED;

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
            // Line Avoidance
            robotState = "Line Avoidance";
            motors.run(moveSpeed, floatMod(ls.lineDirection + 180, 360), 0);
        } else {
            if(dirCalc.attack) {
                // Attacker Logic
                robotState = "Attacker Logic";
                if(tssp.ballDir >= 350 || tssp.ballDir <= 10) {
                    motors.run((tssp.detectingBall?SET_SPEED:0), tssp.ballDir, 
                            correction);
                } else {
                    if(tssp.ballStr > ORBIT_STRENGTH_RADIUS) {
                        motors.run((tssp.detectingBall?moveSpeed:0),
                                attackerMoveDirection, correction);
                    } else {
                        motors.run((tssp.detectingBall?moveSpeed:0), 
                                tssp.ballDir, correction);
                    }
                }
            } else {
                // Defender Logic
                robotState = "Defender Logic";
            }
        }
    #endif

    #if DEBUG_ROBOT_STATE
        Serial.println(robotState);
    #endif
}