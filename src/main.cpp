#include <Arduino.h>
#include <configandpins.h>
#include <drive_system.h>
#include <tssp_system.h>
#include <light_system.h>
#include <DirCalc.h>
#include <PID.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <batread.h>

Drive_system motors;
Tssp_system tssp;
DirectionCalc dirCalc;
bno::Adafruit_BNO055 compass;
PID compass_correct(PID_p, PID_i, PID_d, PID_abs_max);
sensors_event_t rotation;
BatRead batteryLevel;

float correction = 0;
float moveDirection = 0;
float moveSpeed = 0;
float batteryCurrentLevel = 0;

void setup() {
    Serial.begin(9600);
    tssp.init();
    motors.init();
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
    correction = compass_correct.update(rotation.orientation.x > 180 ? rotation.orientation.x - 360 : rotation.orientation.x, 0);
    float batteryCurrentLevel = batteryLevel.read();
    moveDirection = dirCalc.trigOrbit(tssp.ballStr, tssp.ballDir);
    moveSpeed = dirCalc.calcSpeed(tssp.ballStr);
    motors.run(moveSpeed, moveDirection, 0, correction, batteryCurrentLevel, tssp.detectingBall);
}