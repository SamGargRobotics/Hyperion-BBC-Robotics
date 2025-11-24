#include <Arduino.h>
#include <Drive_system.h>
#include <Debug.h>
#include <PID.h>   
#include <TSSP_system.h>
#include <Bluetooth.h>
#include <Kicker.h>
#include <Adafruit_BNO055.h>
#include <Camera.h>

// CAMERA (openmv) - tom
// KICKER (strategy) - tom
// DRIBBLER (mechanics, strategy) - (tom, sam)
// DEBUG SETUP (sam)
// LIGHT SYSTEM (setup) (tom)

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B, &Wire);
PID correction(KP_IMU, 0.0, KD_IMU, 100.0);
PID goalTrack(KP_GOALT, 0.0, KP_GOALT, 100.0);
PID hozt(KP_HOZT, 0.0, KD_HOZT);
PID vert(KP_VERT, 0.0, KD_VERT);
VoltageDivider battery(ROBOT_VD_PIN, ROBOT_VOLTAGE_STABALISER);
Timer batteryTimer(5000000);    
DriveSystem motors;
TsspSystem tssp;
Bluetooth bt;
Camera cam;
sensors_event_t bearing;
LightSystem ls;


bool isSurging = false;

void setup() {
    motors.init();
    tssp.init();
    bt.init();
    cam.init();
    ls.init();
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
    cam.update(false);
    bt.update(tssp.ball().dir(), tssp.ball().str(), cam.attack().angle(), 
              cam.defend().dist(), 0.0f, false);
    ls.inner_circle_direction_calc(bearing.orientation.x,true);
    float _dir = 0;
    float _spd = 0;
    float _cor = -correction.update((bearing.orientation.x > 180) ? 
                                    bearing.orientation.x - 360  : 
                                    bearing.orientation.x, 0.0);
    if(bt.get_role()) {
        _dir = tssp.move().dir();
        _spd = tssp.move().spd();
        _cor = -goalTrack.update((cam.attack().angle() > 180) ? 
                                cam.attack().angle() - 360  : 
                                cam.attack().angle(), 0.0);
    } else {
        if (!isSurging && tssp.ball().str() >= DEF_START_SURGE) {
            isSurging = true;
        } else if (isSurging && tssp.ball().str() < DEF_KEEP_SURGE_UNTIL) {
            isSurging = false;
        }
        if (isSurging && (tssp.ball().dir() < 30 || tssp.ball().dir() > 330)
            && (cam.defend().dist() <= (SP_DEFEND_VERT + 8.0))) {
            _dir = tssp.ball().dir();
            _spd = SURGE_SPEED;
            if(GOAL_TRACKING_TOGGLE & cam.defend().visible()) {
                _cor = goalTrack.update((fmod(cam.defend().angle() + 180.0, 
                                        360.0)) > 180 ? (fmod(
                                        cam.defend().angle() + 180.0, 360.0)) - 
                                        360.0 : (fmod(cam.defend().angle() + 
                                        180.0, 360.0)), 0.0);
            }
        } else if((tssp.ball().dir() < 310 && tssp.ball().dir() > 130)) {
            _dir = tssp.move().dir();
            _spd = tssp.move().spd();
        } else {
            if(cam.defend().visible()) {
                float vertVect = vert.update(cam.defend().visible() - 
                                                SP_DEFEND_VERT, 0);
                float hoztVect = -hozt.update((tssp.ball().str() != 0) ? 
                                                    ((tssp.ball().dir() > 180) ? 
                                                    tssp.ball().dir() - 360 : 
                                                    tssp.ball().dir()) : 
                                                    -((bearing.orientation.x > 180) 
                                                    ? bearing.orientation.x - 360 : 
                                                    bearing.orientation.x), 0.0);
                _dir = fmod(atan2f(hoztVect, vertVect)*RAD_TO_DEG, 360);
                _spd = sqrtf(powf(vertVect, 2) + powf(hoztVect, 2));
                if(GOAL_TRACKING_TOGGLE) {
                    _cor = goalTrack.update((fmod(cam.defend().angle() + 180.0, 
                                            360.0)) > 180 ? (fmod(
                                            cam.defend().angle() + 180.0, 360.0)) - 
                                            360.0 : (fmod(cam.defend().angle() + 
                                            180.0, 360.0)), 0.0);
                }
            } else {
                _dir = 180.0;
                _spd = BASE_SPEED;
            }
        }
    }
    if (battery.get_lvl() <= ROBOT_REQUIRED_VOLT && batteryTimer.time_has_passed_no_update()) {
        _spd = 0;
        _dir = 0;
        _cor = 20;
    } else {
        batteryTimer.update();
    }
    motors.run(_spd, _dir, _cor);
}