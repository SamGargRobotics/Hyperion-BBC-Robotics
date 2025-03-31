#include <Arduino.h>
#include <configandpins.h>
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

Drive_system motors;
Bluetooth bluetooth;
Tssp_system tssp;
LSystem ls;
DirectionCalc dirCalc;
bno::Adafruit_BNO055 compass;
PID compass_correct(PID_p, PID_i, PID_d, PID_abs_max);
sensors_event_t rotation;
BatRead batteryLevel;

float correction = 0;
float attackerMoveDirection = 0;
float defenderMoveDirection = 0;
float moveSpeed = 0;
float batteryCurrentLevel = 0;
float goalDir = 0;
float goalDis = 0;

void setup() {
    Serial.begin(9600);
    tssp.init();
    ls.init();
    motors.init();
    bluetooth.init();
    batteryLevel.init();
    compass.setExtCrystalUse(true);
    while(!compass.begin()) {
        Serial.println("bno ded ;(");
    }
    Serial2.begin(115200); // Camera Serial Communication
}

void loop() {
    compass.getEvent(&rotation); //(rotation.orientation.x)
    tssp.read();
    batteryCurrentLevel = batteryLevel.read();
    correction = compass_correct.update(rotation.orientation.x > 180 ? rotation.orientation.x - 360 : rotation.orientation.x, 0);

    motors.attack = dirCalc.calculateStrategy(bluetooth.otherRobotBallLocation[1], dirCalc.ballDis); //ERRORING BECAUSE OF PERAMS
    ls.calculateLineDirection();
    ls.calculateLineState();
    attackerMoveDirection = dirCalc.trigOrbit(tssp.ballStr, tssp.ballDir);
    defenderMoveDirection = dirCalc.defenderMovement(goalDir, goalDis, tssp.ballDir);
    moveSpeed = dirCalc.calcSpeed(tssp.ballStr);
    motors.run(moveSpeed,(motors.attack?attackerMoveDirection:defenderMoveDirection), 0, correction, batteryCurrentLevel, ls.lineDirection, motors.attack?(goalDir):(dirCalc.defenderRotationOffset), tssp.detectingBall);
} 
/*
if there is NOT connection --> Defend
    else if:
        bat high + ball Closer : Attack
        bat high + ball far : defend
        bat low + ball closer : attack
        bat low + ball far : defend
*/