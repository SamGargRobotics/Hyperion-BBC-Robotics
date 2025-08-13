#include <Drive_system.h>
#include <TSSP_system.h>
#include <Light_system.h>
#include <PID.h>
#include <Adafruit_BNO055.h>
#include <Camera.h>
#include <VoltDiv.h>
#include <Timer.h>
#include <Bluetooth.h>




Adafruit_BNO055 bno;
Bluetooth bt;
Camera cam;
Drive_system motors;
Light_system ls;
PID bearingCorrection(KP_IMU, KI_IMU, KD_IMU, 100.0);
PID camAttackCorrection(KP_CAM_ATTACK, KI_CAM_ATTACK, KD_CAM_ATTACK);
PID camDefendCorrection(KP_CAM_DEFEND, KI_CAM_DEFEND, KD_CAM_DEFEND);
PID defenderVert(KP_DEFEND_VERT, KI_DEFEND_VERT, KD_DEFEND_VERT);
PID defenderHozt(KP_DEFEND_HOZT, KI_DEFEND_HOZT, KD_DEFEND_HOZT, 100);
PID avoidLine(KP_LINE_AVOID, KI_LINE_AVOID, KD_LINE_AVOID, 255.0);
PID centeringPID(KP_CENTERING, KI_CENTERING, KD_CENTERING);
Timer batteryTimer(5000000);    
Tssp_system tssp;
VoltDiv battery(BATT_READ_PIN, BATTERY1_DIVIDER);

Timer testMot(1000000);

sensors_event_t bearing;

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
    testMot.resetTime();
    cam.init();
    bno.setExtCrystalUse(true);
    pinMode(GOAL_PIN, INPUT);
    bno.begin();
    pinMode(COM_PIN, INPUT);
}

void loop() {
    #if COMPETITION_MODE
        bool motorSwitch = (battery.update() > 9.6);
        bool commEnable = digitalRead(COM_PIN);
    #else
        bool motorSwitch = true;
        bool commEnable = true;
    #endif

    tssp.update();
    bt.update(tssp.getBallDir(), tssp.getBallStr(), (motorSwitch && commEnable));

    bno.getEvent(&bearing);
    float heading = (bearing.orientation.x > 180) ? bearing.orientation.x - 360
                    : bearing.orientation.x;
    cam.update(digitalRead(GOAL_PIN));
    ls.update(bearing.orientation.x, (motorSwitch && commEnable));

    float moveDir = 0.0;
    float moveSpeed = 0.0;
    float bearingCor = -bearingCorrection.update(heading, 0.0);
    float correction = bearingCor;

    float modBallDir = tssp.getBallDir() > 180 ? tssp.getBallDir() - 360
                            : tssp.getBallDir();
    float moveScaler = constrain(tssp.getBallStr() /
                    ORBIT_STRENGTH_RADIUS, 0, 1);
    moveScaler = constrain((0.02 * moveScaler * expf(4.5 * moveScaler)), 0, 1);
    float moveOffset = moveScaler * min(0.4 * expf(0.25 * abs(modBallDir))
                    - 0.4, 90.0);
    if(true) {
        if(tssp.getBallStr() != 0) {
            // moveDir = floatMod((modBallDir < 0 ? -moveOffset : moveOffset) + tssp.getBallDir(), 360.0);
            moveDir = (tssp.getBallDir() > 180)?(tssp.getBallDir() + (-min(0.04*(expf(-4.5*(tssp.getBallDir()-360)) - 1), ORBIT_STRENGTH_RADIUS))):(tssp.getBallDir() + (min(0.04*(expf(-4.5*(tssp.getBallDir()-360)) - 1), ORBIT_STRENGTH_RADIUS)));
            moveSpeed = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0 - moveOffset / 90.0);
            //OLD MOVESPD: moveSpeed = BASE_SPEED;
            correction = bearingCor;

            //HARD CODING SURGE (not intergrated): (REMOVE ONCE ORBIT IS TUNED)
            if(tssp.getBallDir() > 340 || tssp.getBallDir() < 20) {
                // moveDir = cam.getAttackGoalAngle();
                moveDir = tssp.getBallDir();
                moveSpeed = SURGE_SPEED;
                if(cam.getAttackGoalVisible()) {
                    // float headingMulti = 0.0011985*pow(1.08226, cam.getAttackGoalDist());
                    float goalHeading = cam.getAttackGoalAngle() > 180 ?
                                        cam.getAttackGoalAngle() - 360 :
                                        cam.getAttackGoalAngle();
                    // goalHeading *= headingMulti;
                    correction = camAttackCorrection.update(goalHeading, 0.0);
                }
            }
        } else {
            // allows the robot to centre on the field when not seeing the ball
            // moveDir = (heading > 0)? 90 : 270;
            // moveSpeed = centeringPID.update(heading, 0.0);
            // moveSpeed = (moveSpeed > SURGE_SPEED)?SURGE_SPEED:moveSpeed;
            // Serial.println(moveSpeed);
            moveDir = 0;
            moveSpeed = 0;
        }
        #if GOAL_TRACKING_TOGGLE
            // if(cam.getAttackGoalVisible()) {
            //     float goalHeading = cam.getAttackGoalAngle() > 180 ?
            //                         cam.getAttackGoalAngle() - 360 :
            //                         cam.getAttackGoalAngle();
            //     correction = camAttackCorrection.update(goalHeading, 0.0);
            // }
        #endif
    } else {
        if((tssp.getBallDir() > 90 && tssp.getBallDir() <= 270) || ((tssp.getBallDir() < 15 || tssp.getBallDir() > 345) && tssp.getBallStr() > DEFEND_SURGE)) {
            moveDir = floatMod((modBallDir < 0 ? -moveOffset : moveOffset) + tssp.getBallDir(), 360.0);
            moveSpeed = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0 - moveOffset / 90.0);
        } else {
            if(cam.getDefendGoalVisible()) {
                float vertVect = -defenderVert.update(abs(cam.getDefendGoalDist()), 75.0);
                float defHeading = (tssp.getBallDir() > 180) ? tssp.getBallDir() - 360 : tssp.getBallDir();
                float hoztVect = -defenderHozt.update(defHeading, 0.0);
                moveDir = floatMod(atan2(hoztVect, vertVect)*RAD_TO_DEG, 360.0);
                moveSpeed = sqrt(pow(vertVect, 2)+pow(hoztVect, 2));
                if(moveSpeed > SURGE_SPEED) {
                    moveSpeed = SURGE_SPEED;
                }
                Serial.print(moveDir);
                Serial.print("\t");
                Serial.print(moveSpeed);
                Serial.print("\t");
            }
            #if GOAL_TRACKING_TOGGLE
                if(cam.getDefendGoalVisible()) {
                    // float target = floatMod(cam.getDefendGoalAngle() + 180.0, 360.0);
                    // float goalHeading = target > 180 ? target - 360.0 : target;
                    // correction = camDefendCorrection.update(goalHeading, 0.0);
                    float goalHeading = cam.getAttackGoalAngle() > 180 ?
                                        cam.getAttackGoalAngle() - 360 :
                                        cam.getAttackGoalAngle();
                    correction = camDefendCorrection.update(goalHeading, 180.0);
                }
            #endif
        }
    }
    
    if(ls.getLineState() > ATK_LINE_SP) {
        moveDir = floatMod(ls.getLineDirection() + 180, 360.0);
        moveSpeed = -avoidLine.update(ls.getLineState(), 0.0);
    }

    float modGoalAngle = cam.getAttackGoalAngle() > 180?cam.getAttackGoalAngle() - 360 : cam.getAttackGoalAngle();
    if(abs(modGoalAngle) > 15) {
        moveSpeed = moveSpeed/1.2;
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
    #if not GOAL_TRACKING_TOGGLE
        correction = bearingCor; 
    #endif
    if(ls.getLineDirection() != -1) {
        correction = bearingCor;
    }
    Serial.print(moveDir);
    Serial.print("\t");
    Serial.println(moveSpeed);
    if(motorSwitch && commEnable) {
        motors.run(moveSpeed, moveDir, correction);
        // motors.run(80, 0, 0);
    } else {
        motors.run(0, 0, 0);
    }
}