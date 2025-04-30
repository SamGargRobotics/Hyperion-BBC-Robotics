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

Drive_system motors;
Bluetooth bluetooth;
Tssp_system tssp;
LSystem ls;
Camera cam;
DirectionCalc dirCalc;
bno::Adafruit_BNO055 compass = bno::Adafruit_BNO055(-1, 0x29, &Wire);
PID compass_correct(PID_p, PID_i, PID_d, PID_abs_max);
sensors_event_t rotation;
BatRead batteryLevel;

//! @brief Amount needed to turn (degs) to ensure that you are goal tracking/
//!        staying forward depending on case.
float correction = 0;
//! @brief The amount the robot must correct to stay forward in terms of camera
float goalTrackingCorrection = 0;
//! @brief Goal tracking correction for blue goal
float blueGoalTarget = 0;
//! @brief Goal tracking correction for yellow goal
float yellowGoalTarget = 0;
//! @brief The amount the robot must correct to stay forward in terms of compass
float regularTrackingCorrection = 0;
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
//! @brief If the robot is in the front quadrant of the field
bool robotFrontQuadrant = false;
//! @brief If the motor switch is turned on
bool motorOn = false;

void setup() {
    Serial.begin(9600);
    tssp.init();
    ls.init();
    motors.init();
    bluetooth.init();
    batteryLevel.init();
    compass.begin();
    compass.setExtCrystalUse(true);
    cam.init();
    pinMode(BAT_LED_PIN, OUTPUT);
    while(!compass.begin()) {
        Serial.println("bno ded ;(");
    }
}

void loop() {
    compass.getEvent(&rotation); //(rotation.orientation.x)
    tssp.normalCalc();
    // tssp.advancedCalc();
    batteryLevel.read();
    cam.read_camera();
    blueGoalTarget = floatMod(-1*cam.angle_to_goal_blue, 360) > 180 ? \
                     floatMod(-1*cam.angle_to_goal_blue, 360) - 360 : \
                     floatMod(-1*cam.angle_to_goal_blue, 360);
    yellowGoalTarget = floatMod(-1*cam.angle_to_goal_yellow, 360) > 180 ? \
                       floatMod(-1*cam.angle_to_goal_yellow, 360) - 360 : \
                       floatMod(-1*cam.angle_to_goal_yellow, 360);
    goalTrackingCorrection = -1*compass_correct.update((attackingGoal) ? \
                             blueGoalTarget:yellowGoalTarget, 0);
    regularTrackingCorrection = -1*compass_correct.update( \
                                rotation.orientation.x > 180 ? \
                                rotation.orientation.x - 360 : \
                                rotation.orientation.x, 0);

    #if GOAL_TRACKING_TOGGLE
        if(goalTrackingCorrection <= -90 || goalTrackingCorrection >= 90) {
            correction = regularTrackingCorrection;
        } else {
            correction = goalTrackingCorrection;
        }
    #else
        correction = regularTrackingCorrection;
    #endif

    robotFrontQuadrant = (cam.goal_y_blue < cam.goal_y_yellow);
    motors.attack = dirCalc.calculateStrategy(
                    bluetooth.otherRobotBallLocation[1], dirCalc.ballDis);
    attackerMoveDirection = dirCalc.exponentialOrbit(tssp.normalBallDir, 
                                                    tssp.ballStr);
    defenderMoveDirection = dirCalc.defenderMovement(0, 0, 
                                                    tssp.normalBallDir);                      
    moveSpeed = dirCalc.calcSpeed(tssp.ballStr)*SET_SPEED;
    if(batteryLevel.volts <= BATTERY_CRITICAL) {
        digitalWrite(BAT_LED_PIN, HIGH);
    } else {
        digitalWrite(BAT_LED_PIN, LOW);
    }

    #if DEBUG_IMU
        Serial.print(rotation.orientation.x);
        Serial.print(" ");
        Serial.println(correction);
    #endif

    #if DEBUG_TSSP
        Serial.print(tssp.normalBallDir);
        Serial.print("\t");
        Serial.print(tssp.ballStr);
        Serial.print("\t");
        Serial.print(tssp.detectingBall);
        Serial.print("\t");
        Serial.print(moveSpeed);
        Serial.print("\t");
        Serial.print(attackerMoveDirection);
        Serial.print("\t");
        Serial.println(tssp.ballStr);
    #endif

    #if BAT_READ_VOLTS
        if(batteryLevel.motorOn) {
            Serial.print("MOTOR ON");
        } else {
            Serial.print("MOTOR OFF");
        }
        Serial.print("\t");
        Serial.println(batteryLevel.volts);
    #endif
    
    #if DEBUG_GOAL_TRACKING
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
        Serial.println(cam.angle_to_goal_yellow);
        Serial.print("\t"); 
    #endif

    #if CORRECTION_TEST
        motors.run(0, 0, correction);
    #elif BALL_FOLLOW_TEST
        motors.run((tssp.detectingBall?moveSpeed:0), 
                  tssp.normalBallDir, 0);
    #else
        if(tssp.normalBallDir >= 350 || tssp.normalBallDir <= 10) {
            motors.run((tssp.detectingBall?moveSpeed:0),
                        tssp.normalBallDir, correction);
        } else {
            if(tssp.ballStr > ORBIT_STRENGTH_RADIUS) {
                motors.run((tssp.detectingBall?moveSpeed:0),
                          attackerMoveDirection, correction);
            } else {
                motors.run((tssp.detectingBall?moveSpeed:0), 
                          tssp.normalBallDir, correction);
            }

        }
    #endif
}