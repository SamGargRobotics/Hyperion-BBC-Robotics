#include <Drive_system.h>
#include <TSSP_system.h>
#include <Light_system.h>
#include <PID.h>
#include <Adafruit_BNO055.h>
#include <Camera.h>
#include <VoltDiv.h>
#include <Timer.h>
#include <Bluetooth.h>




Adafruit_BNO055 bno(-1, 0x29, &Wire);
Bluetooth bt;
Camera cam;
Drive_system motors;
Light_system ls;
PID bearingCorrection(KP_IMU, KI_IMU, KD_IMU);
PID camAttackCorrection(KP_CAM_ATTACK, KI_CAM_ATTACK, KD_CAM_ATTACK);
PID camDefendCorrection(KP_CAM_DEFEND, KI_CAM_DEFEND, KD_CAM_DEFEND);
PID defenderVert(KP_DEFEND_VERT, KI_DEFEND_VERT, KD_DEFEND_VERT);
PID defenderHozt(KP_DEFEND_HOZT, KI_DEFEND_HOZT, KD_DEFEND_HOZT);
PID avoidLine(KP_LINE_AVOID, KI_LINE_AVOID, KD_LINE_AVOID);
PID centeringPID(KP_CENTERING, KI_CENTERING, KD_CENTERING);
Timer batteryTimer(5000000);    
Tssp_system tssp;
VoltDiv battery(BATT_READ_PIN, BATTERY1_DIVIDER);




void setup() {
    #if DEBUG
        Serial.begin(9600);
    #endif
    tssp.init();
    ls.init();
    motors.init();
    bt.init();
    battery.init();
    batteryTimer.resetTime();
    cam.init();
    bno.setExtCrystalUse(true);
    pinMode(GOAL_PIN, INPUT);
    bno.begin();
}

void loop() {
    tssp.update();
    bt.update(tssp.getBallDir(), tssp.getBallStr());

    sensors_event_t bearing;
    bno.getEvent(&bearing);
    float heading = (bearing.orientation.x > 180) ? bearing.orientation.x - 360
                    : bearing.orientation.x;
    cam.update(digitalRead(GOAL_PIN));
    ls.update(bearing.orientation.x);

    float moveDir = 0.0;
    float moveSpeed = 0.0;
    float correction = -bearingCorrection.update(heading, 0);

    if(true) {//bt.getRole()) {
        // Role --> Attacking
        if(tssp.getBallStr() != 0) {
            // Ball is visible --> Ball Movement
            float modBallDir = tssp.getBallDir() > 180 ? tssp.getBallDir() - 360
                               : tssp.getBallDir();
            float moveScaler = constrain(tssp.getBallStr() /
                               ORBIT_STRENGTH_RADIUS, 0, 1);
            moveScaler = constrain((0.02 * moveScaler * expf(4.5 * moveScaler)), 0, 1);
            float moveOffset = moveScaler * min(0.4 * expf(0.25 * abs(modBallDir))
                               - 0.4, 90.0);
            moveDir = floatMod((modBallDir < 0 ? -moveOffset : moveOffset) + tssp.getBallDir(), 360.0);
            moveSpeed = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0 -
                        moveOffset / 90.0);
            if(cam.getAttackGoalVisible()) {
                float goalHeading = cam.getAttackGoalAngle() > 180 ?
                                    cam.getAttackGoalAngle() - 360 :
                                    cam.getAttackGoalAngle();
                correction = camAttackCorrection.update(goalHeading, 0.0);
            }
        } else {
            // Ball is not visible --> Center on field
            if(cam.getAttackGoalVisible()) {
                moveDir = (bearing.orientation.x < 360 && bearing.orientation.x
                          > 180)? 90 : 270;
                moveSpeed = centeringPID.update(heading, 0);
                float goalHeading = cam.getAttackGoalAngle() > 180 ?
                                    cam.getAttackGoalAngle() - 360 :
                                    cam.getAttackGoalAngle();
                correction = camAttackCorrection.update(goalHeading, 0.0);
            }
        }
        if(ls.getLineState() > 0.0) {
            // If seeing line --> avoid line
            if(smallestAngleBetween(ls.getLineDirection(), moveDir) < 45.0 ||
               ls.getLineState() > LINE_STRICT_AVOID) {
                moveDir = floatMod(ls.getLineDirection() + 180, 360.0);
                moveSpeed = -avoidLine.update(ls.getLineState(), ATK_LINE_SP);
            }
        }
    } else {
        // Role --> Defending
        if(tssp.getBallDir() > 110 && tssp.getBallDir() <= 290) {
            // Ball is behind --> Move backwards
            moveDir = 180;
            moveSpeed = BASE_SPEED;
        } else {
            // Ball is infront --> Defend goal.
            float vertVect = -defenderVert.update(ls.getLineState(),
                                                  DEF_VERT_SP);
            float defHeading = (tssp.getBallStr() != 0)?((tssp.getBallDir()
                                > 180)?tssp.getBallDir() - 360 :
                                tssp.getBallDir()) : -heading;
            float hoztVect = -defenderHozt.update(defHeading, 0);
            moveDir = floatMod(atan2f(hoztVect, vertVect)*RAD_TO_DEG, 360);
            moveSpeed = sqrtf(powf(vertVect, 2) + powf(hoztVect, 2));
        }
        if(cam.getDefendGoalVisible()) {
            float goalHeading = cam.getDefendGoalAngle() > 180 ?
                                cam.getDefendGoalAngle() - 360 :
                                cam.getDefendGoalAngle();
            correction = -camDefendCorrection.update(goalHeading, 0);
        } else {
            correction = -bearingCorrection.update(heading, 0);
        }
        // If seeing line, only avoid line if in corner.
        if(ls.getLineState() > 0.0) {
            if(smallestAngleBetween(ls.getLineDirection(), moveDir) < 60.0 &&
               ls.getLineState() > LINE_STRICT_AVOID) {
                moveDir = floatMod(ls.getLineDirection() + 180, 360.0);
                moveSpeed = -avoidLine.update(ls.getLineState(), DEF_LINE_SP);
            }
        }
    }
    #if not COMPETITION_MODE
        if(battery.update() <= BATTERY_CRITICAL) {
            // When battery is low, spin in small circle (TESTING ONLY).
            if(batteryTimer.timeHasPassedNoUpdate()) {
                moveSpeed = 0;
                correction = 20;
            }
        } else {
            batteryTimer.resetTime();
        }
    #endif

    // Run motors depending on given values through logic.
    motors.run(moveSpeed, moveDir, correction);
}