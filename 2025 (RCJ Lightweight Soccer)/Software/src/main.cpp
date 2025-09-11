#include <Drive_system.h>
#include <Light_system.h>
#include <Adafruit_BNO055.h>
#include <VoltDiv.h>
#include <Timer.h>
#include <Bluetooth.h>
#include <Camera.h>
#include <TSSP_system.h>
#include <Debug.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B, &Wire);
Bluetooth bt;
Camera cam;
Drive_system motors;
Light_system ls;
PID avoidLine(KP_LINE_AVOID, KI_LINE_AVOID, KD_LINE_AVOID, 255.0);
PID defendHozt(KP_DEFEND_HOZT, KI_DEFEND_HOZT, KD_DEFEND_HOZT);
PID defendVert(KP_DEFEND_VERT, KI_DEFEND_HOZT, KD_DEFEND_HOZT);
PID attackCor(KP_CAM_ATTACK, KI_CAM_ATTACK, KD_CAM_ATTACK);
PID defendCor(KP_CAM_DEFEND, KI_CAM_DEFEND, KD_CAM_DEFEND);
PID bearingCor(KP_IMU, KI_IMU, KD_IMU, 100.0);
PID centeringPID(1.5, KI_DEFEND_HOZT, KD_DEFEND_HOZT);
PID centeringPIDVert(7.0, KI_DEFEND_HOZT, KD_DEFEND_HOZT);
Timer batteryTimer(5000000);
Tssp_system tssp;
VoltDiv battery(BATT_READ_PIN, BATTERY1_DIVIDER);
sensors_event_t bearing;
Debug debug;

void setup() {
    debug.init(9600);
    if(DEBUG_CORE_CODE) {
        // Type any of the below strings into the terminal (when monitoring) and
        // the program sends out the corresponding value
        debug.addMode("ballDir", []() { return String(tssp.getBallDir()); });
        debug.addMode("ballStr", []() { return String(tssp.getBallStr()); });
        debug.addMode("compass", []() { return String(bearing.orientation.x); });
        debug.addMode("switch", []() { return String(bt.getRole()); });
        debug.addMode("camAtkAng", []() { return String(cam.getAttackGoalAngle()); });
        debug.addMode("camAtkDis", []() { return String(cam.getAttackGoalDist()); });
        debug.addMode("camDefAng", []() { return String(cam.getDefendGoalAngle()); });
        debug.addMode("camDefDis", []() { return String(cam.getDefendGoalDist()); });
        debug.addMode("lsDir", []() { return String(ls.getLineDirection()); });
        debug.addMode("lsState", []() { return String(ls.getLineState()); });
        // debug.addMode("batLvl", []() { return String(battery.update()); });

        // After you are done monitoring that specific variable and want to do
        // another, you can simply say "stop" in the terminal, and it stops.
        
        // If you want to see multiple things at the same time, you can seperate
        // them by commas, for example:
        // "switch,camAtkDis,camDefDis" --> outputs the corresponding functions
    }

    tssp.init();
    ls.init();
    motors.init();
    bt.init();
    battery.init();
    batteryTimer.resetTime();
    cam.init();
    pinMode(GOAL_PIN, INPUT);
    while(!bno.begin(OPERATION_MODE_IMUPLUS)) {
        Serial.println("No BNO055 detected. Check your wiring or I2C ADDR.");
        delay(1000);
    }
    delay(500);
    bno.setExtCrystalUse(true);
    delay(500);
}

void loop() {
    tssp.update();
    bt.update(tssp.getBallDir(), tssp.getBallStr(), true);
    bno.getEvent(&bearing);
    cam.update(digitalRead(GOAL_PIN));
    ls.update(bearing.orientation.x, true);
    float moveDir = 0.0;
    float moveSpeed = 0.0;
    float correction = -bearingCor.update((bearing.orientation.x > 180) ? 
                                            bearing.orientation.x - 360
                                            : bearing.orientation.x, 0.0);

    if(!SECOND_ROBOT) {
        if(tssp.getBallStr() == 0) {
            if(cam.getAttackGoalVisible()) {
                float set = cam.getAttackGoalAngle() > 180.0 ? cam.getAttackGoalAngle() - 360.0 :
                    cam.getAttackGoalAngle();
                float otherSet = 47.0 - cam.getAttackGoalDist();
                float hoztVect = -centeringPID.update(set, 0.0);
                float vertVect = centeringPIDVert.update(otherSet, 0.0);
                moveDir = floatMod(atan2f(hoztVect, vertVect) * RAD_TO_DEG, 360.0);
                moveSpeed = sqrtf(powf(vertVect, 2) + powf(hoztVect, 2));
            }
        } else {
            if(cam.getAttackGoalVisible() && GOAL_TRACKING_TOGGLE && !(tssp.getBallDir() > 50 && tssp.getBallDir() < 310)) {
                correction = attackCor.update(cam.getAttackGoalAngle() > 180.0 ? cam.getAttackGoalAngle() - 360.0 :
                    cam.getAttackGoalAngle(), 0.0);
            }
            tssp.orbit();
            moveDir = tssp.getMoveDir();
            moveSpeed = tssp.getMoveSpd();
        }
    } else {
        if((tssp.getBallStr() >= DEFEND_SURGE && (tssp.getBallDir() < 30 || tssp.getBallDir() > 330)) || (tssp.getBallDir() < 270 && tssp.getBallDir() > 90)) {
            tssp.orbit();
            moveDir = tssp.getMoveDir();
            // moveSpeed = tssp.getMoveSpd()/2 < 30.0 ? 30.0 : tssp.getMoveSpd()/2;
            moveSpeed = tssp.getMoveSpd();
            if(cam.getAttackGoalVisible() && GOAL_TRACKING_TOGGLE && !(tssp.getBallDir() > 50 && tssp.getBallDir() < 310)) {
                correction = attackCor.update(cam.getAttackGoalAngle() > 180.0 ? cam.getAttackGoalAngle() - 360.0 :
                    cam.getAttackGoalAngle(), 0.0);
            }
        } else {
            float vertVect = defendVert.update(cam.getDefendGoalDist() - 30.0, 0);
            float defHeading = (tssp.getBallStr() != 0) ? ((tssp.getBallDir() > 180) ? tssp.getBallDir() - 360 : tssp.getBallDir()) : -((bearing.orientation.x > 180) ? bearing.orientation.x - 360 : bearing.orientation.x);
            float hoztVect = -defendHozt.update(defHeading, 0.0);
            // hoztVect = 0;
            if(cam.getDefendGoalVisible()) {
                moveDir = floatMod(atan2f(hoztVect, vertVect)*RAD_TO_DEG, 360);
            } else {
                moveDir = 180.0;
                moveSpeed = 80.0;
            }
            moveSpeed = sqrtf(powf(vertVect, 2) + powf(hoztVect, 2));
            if(cam.getDefendGoalVisible() && GOAL_TRACKING_TOGGLE) {
                float target = floatMod(cam.getDefendGoalAngle() + 180.0, 360.0);
                correction = defendCor.update(target > 180 ? target - 360.0 : 
                                    target, 0.0);
            }
        }
    }

    if(ls.getLineState() > ATK_LINE_SP) {
        moveDir = floatMod(ls.getLineDirection() + 180.0, 360.0);
        if(smallestAngleBetween(moveDir, ls.getLineDirection()) < 90) {
            moveSpeed = -avoidLine.update(ls.getLineState(), ATK_LINE_SP);
        } else {
            moveSpeed = BASE_SPEED;
        }
    }

    if(DEBUG_CORE_CODE) {
        debug.update();
    }

    if (battery.update() <= BATTERY_CRITICAL && batteryTimer.timeHasPassedNoUpdate()) {
        moveSpeed = 0;
        correction = 20;
    } else {
        batteryTimer.resetTime();
    }

    motors.run(moveSpeed, moveDir, correction);
}