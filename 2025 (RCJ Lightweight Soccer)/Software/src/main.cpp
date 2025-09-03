#include <Drive_system.h>
#include <TSSP_system.h>
#include <Light_system.h>
#include <Adafruit_BNO055.h>
#include <Camera.h>
#include <VoltDiv.h>
#include <Timer.h>
#include <Bluetooth.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B, &Wire);
Bluetooth bt;
Camera cam;
Drive_system motors;
Light_system ls;
PID avoidLine(KP_LINE_AVOID, KI_LINE_AVOID, KD_LINE_AVOID, 255.0);
PID centeringPID(KP_CENTERING, KI_CENTERING, KD_CENTERING);
PID defendHozt(KP_DEFEND_HOZT, KI_DEFEND_HOZT, KD_DEFEND_HOZT);
PID defendVert(KP_DEFEND_VERT, KI_DEFEND_HOZT, KD_DEFEND_HOZT);
PID attackCor(KP_CAM_ATTACK, KI_CAM_ATTACK, KD_CAM_ATTACK);
PID defendCor(KP_CAM_DEFEND, KI_CAM_DEFEND, KD_CAM_DEFEND);
PID bearingCor(KP_IMU, KI_IMU, KD_IMU, 100.0);
Timer batteryTimer(5000000);
Tssp_system tssp;
VoltDiv battery(BATT_READ_PIN, BATTERY1_DIVIDER);
sensors_event_t bearing;

void setup() {
    Serial.begin(9600);
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
    bool role = true;
    tssp.update();
    bt.update(tssp.getBallDir(), tssp.getBallStr(), true);
    bno.getEvent(&bearing);
    cam.update(false);
    ls.update(bearing.orientation.x, true);
    float moveDir = 0.0;
    float moveSpeed = 0.0;
    float correction = -bearingCor.update((bearing.orientation.x > 180) ? 
                                            bearing.orientation.x - 360
                                            : bearing.orientation.x, 0.0);

    if(role) {
        if(tssp.getBallStr() != 0) {
            float modBallDir = tssp.getBallDir() > 180 ? tssp.getBallDir() - 360
                : tssp.getBallDir();
            float moveScaler = constrain(tssp.getBallStr() /
                            ORBIT_STRENGTH_RADIUS, 0, 1);
            moveScaler = constrain((0.02 * moveScaler * expf(4.5 * moveScaler)), 0, 1);
            float moveOffset = moveScaler * min(0.4 * expf(0.25 * abs(modBallDir))
                            - 0.4, 90.0);
            moveDir = floatMod((modBallDir < 0 ? -moveOffset : moveOffset) + tssp.getBallDir(), 360.0);
            moveSpeed = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0 - moveOffset / 90.0);
            if((tssp.getBallDir() < 30.0 || tssp.getBallDir() > 330.0) && tssp.getBallStr() > 110.0) {
                moveDir = tssp.getBallDir();
                moveSpeed = SURGE_SPEED+20;
            }
        } else {
            moveDir = 0.0;
            moveSpeed = 0.0;
        }
        if(cam.getAttackGoalVisible() && GOAL_TRACKING_TOGGLE && !(tssp.getBallDir() > 50 && tssp.getBallDir() < 310)) {
            correction = attackCor.update(cam.getAttackGoalAngle() > 180.0 ? cam.getAttackGoalAngle() - 360.0 :
                cam.getAttackGoalAngle(), 0.0);
        }
    } else {
        if(tssp.getBallStr() >= ORBIT_STRENGTH_RADIUS) {
            if(tssp.getBallStr() != 0) {
                float modBallDir = tssp.getBallDir() > 180 ? tssp.getBallDir() - 360 : tssp.getBallDir();
                float moveScaler = constrain(tssp.getBallStr() / ORBIT_STRENGTH_RADIUS, 0, 1);
                moveScaler = constrain((0.02 * moveScaler * expf(4.5 * moveScaler)), 0, 1);
                float moveOffset = moveScaler * min(0.4 * expf(0.25 * abs(modBallDir))
                                - 0.4, 90.0);
                moveDir = floatMod((modBallDir < 0 ? -moveOffset : moveOffset) + tssp.getBallDir(), 360.0);
                moveSpeed = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0 - moveOffset / 90.0);
                if((tssp.getBallDir() < 30.0 || tssp.getBallDir() > 330.0) && tssp.getBallStr() > 110.0) {
                    moveDir = tssp.getBallDir();
                    moveSpeed = SURGE_SPEED+20;
                };
            } else {
                moveDir = 0.0;
                moveSpeed = 0.0;
            }
        } else {
            float vertVect = 0;
            float defHeading = (tssp.getBallDir() > 180) ? tssp.getBallDir() - 360 :
                                tssp.getBallDir();
            float hoztVect = -defendHozt.update(defHeading, 0.0);
            moveDir = floatMod(atan2f(hoztVect, vertVect)*RAD_TO_DEG, 360);
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

    if((battery.update() <= BATTERY_CRITICAL) && !COMPETITION_MODE) {
        if(batteryTimer.timeHasPassedNoUpdate()) {
            moveSpeed = 0;
            correction = 20;
        }
    } else {
        batteryTimer.resetTime();
    }
    
    motors.run(moveSpeed, moveDir, correction);
}