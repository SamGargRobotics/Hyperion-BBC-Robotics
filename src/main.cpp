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

float correction = 0;
float attackerMoveDirection = 0;
float defenderMoveDirection = 0;
float moveSpeed = 0;
float batteryCurrentLevel = 0;
float currentBatteryLevelVolts = 0;
float goalDir = 0;
float goalDis = 0;
bool robotFrontQuadrant = false;
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
    // while(!compass.begin()) {
    //     Serial.println("bno ded ;(");
    // }
}

void loop() {
    compass.getEvent(&rotation); //(rotation.orientation.x)
    tssp.normalCalc();
    // tssp.advancedCalc();
    batteryLevel.read();
    cam.read_camera();
    correction = -1*compass_correct.update(rotation.orientation.x > 180 ? \
                 rotation.orientation.x - 360 : rotation.orientation.x, 0) + cam.angle_to_goal_yellow;

    // robotFrontQuadrant = (cam.goal_y_attack < cam.goal_y_defend);

    motors.attack = dirCalc.calculateStrategy(
                    bluetooth.otherRobotBallLocation[1], dirCalc.ballDis);

    // ls.calculateLineDirection();
    // ls.calculateLineState();

    attackerMoveDirection = dirCalc.exponentialOrbit(tssp.normalBallDir, 
                                                    tssp.ballStr);
    defenderMoveDirection = dirCalc.defenderMovement(goalDir, goalDis, 
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
    Serial.print(cam.goal_x_blue);
    Serial.print("\t");
    Serial.print(cam.goal_y_blue);
    Serial.print("\t"); 
    Serial.println(cam.angle_to_goal_blue);

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