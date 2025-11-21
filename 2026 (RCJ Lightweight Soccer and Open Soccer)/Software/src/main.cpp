#include <Arduino.h>
#include <Drive_system.h>
#include <Debug.h>
#include <PID.h>   
#include <TSSP_system.h>
#include <Bluetooth.h>
#include <Kicker.h>
#include <Adafruit_BNO055.h>

// CAMERA (openmv, teensy)
// KICKER (strategy)
// DRIBBLER (mechanics, strategy)
// DEBUG SETUP
// ROBOT BATTERY VD SETUP
// LIGHT SYSTEM (setup & library)

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B, &Wire);
PID correction(KP_IMU, 0.0, KD_IMU, 100.0);
DriveSystem motors;
TsspSystem tssp;
Bluetooth bt;

sensors_event_t bearing;

void setup() {
    motors.init();
    tssp.init();
    bt.init();
    while(!bno.begin(OPERATION_MODE_IMUPLUS)) {
        Serial.println("No BNO055 detected. Check your wiring or I2C ADDR.");
        delay(1000);
    }
    delay(500);
    bno.setExtCrystalUse(true);
    delay(500);
}

void loop() {
    bno.getEvent(&bearing);
    tssp.update();
    bt.update(tssp.get_ball_dir(), tssp.get_ball_str(), 0.0f, 0.0f, 0.0f, 0.0f);
    float _dir = 0;
    float _spd = 0;
    float _cor = -correction.update((bearing.orientation.x > 180) ? 
                                    bearing.orientation.x - 360  : 
                                    bearing.orientation.x, 0.0);
    if(bt.get_role()) {
        _dir = tssp.get_move_dir();
        _spd = tssp.get_move_spd();
    }
    motors.run(_spd, _dir, _cor);
}