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
    float correction = -bearingCorrection.update(heading, 0.0);
    // float correction = 0;

    float modBallDir = tssp.getBallDir() > 180 ? tssp.getBallDir() - 360
                            : tssp.getBallDir();
    float moveScaler = constrain(tssp.getBallStr() /
                    ORBIT_STRENGTH_RADIUS, 0, 1);
    moveScaler = constrain((0.02 * moveScaler * expf(4.5 * moveScaler)), 0, 1);
    float moveOffset = moveScaler * min(0.4 * expf(0.25 * abs(modBallDir))
                    - 0.4, 90.0);
    
    if(true) {//bt.getRole()) {
        if(tssp.getBallStr() != 0) {
            moveDir = floatMod((modBallDir < 0 ? -moveOffset : moveOffset) + tssp.getBallDir(), 360.0);
            // moveDir = (tssp.getBallDir() > 180)?(tssp.getBallDir() + (-min(0.04*(expf(-4.5*(tssp.getBallDir()-360)) - 1), 80))):(tssp.getBallDir() + (min(0.04*(expf(-4.5*(tssp.getBallDir()-360)) - 1), 80)));
            // moveSpeed = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0 - moveOffset / 90.0);
            moveSpeed = BASE_SPEED;
            if(tssp.getBallDir() > 320 || tssp.getBallDir() < 40) {
                moveDir = tssp.getBallDir();
                moveSpeed = 100;
            }
            // if(cam.getAttackGoalVisible()) {
            //     float goalHeading = cam.getAttackGoalAngle() > 180 ?
            //                         cam.getAttackGoalAngle() - 360 :
            //                         cam.getAttackGoalAngle();
            //     correction = camAttackCorrection.update(goalHeading, 0.0);
            // }
            // Serial.print(moveDir);
            // Serial.print("\t");
            // Serial.println(tssp.getBallDir());
        }
    } else {
        if(tssp.getBallStr() != 0) {
            if((tssp.getBallDir() > 90 && tssp.getBallDir() <= 270) || ((tssp.getBallDir() < 15 || tssp.getBallDir() > 345) && tssp.getBallStr() > DEFEND_SURGE)) {
                moveDir = floatMod((modBallDir < 0 ? -moveOffset : moveOffset) + tssp.getBallDir(), 360.0);
                moveSpeed = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0 - moveOffset / 90.0);
            } else {
                float vertVect = 0;
                if(ls.getLineState() == 0) {
                    vertVect = -30;
                }
                float defHeading = (tssp.getBallDir() > 180) ? tssp.getBallDir() - 360 :
                                    tssp.getBallDir();
                float hoztVect = -defenderHozt.update(defHeading, 0.0);
                moveDir = floatMod(atan2f(hoztVect, vertVect)*RAD_TO_DEG, 360);
                moveSpeed = sqrtf(powf(vertVect, 2) + powf(hoztVect, 2));
                // #if SECOND_ROBOT
                //     if(cam.getDefendGoalVisible()) {
                //         float target = floatMod(cam.getDefendGoalAngle() + 180.0, 360.0);
                //         float goalHeading = target > 180 ? target - 360.0 : target;
                //         correction = camDefendCorrection.update(goalHeading, 0.0);
                //     }
                // #endif
            }
        } else if(ls.getLineState() == 0) {
            moveDir = 180;
            moveSpeed = 50;
        }
    }
    if(ls.getLineState() > 0.5) {
        moveDir = floatMod(ls.getLineDirection() + 180, 360.0);
        moveSpeed = -avoidLine.update(ls.getLineState(), 0.0);
    } else if(ls.getLineState() > 0 && smallestAngleBetween(moveDir, ls.getLineDirection()) < 45) {
        if(smallestAngleBetween(floatMod(ls.getLineDirection() - 90, 360.0), moveDir) < smallestAngleBetween(floatMod(ls.getLineDirection() + 90, 360.0), moveDir)) {
            moveDir = floatMod(ls.getLineDirection() - 90, 360.0);
            
        } else {
            moveDir = floatMod(ls.getLineDirection() + 90, 360.0);
        }
        moveSpeed *= sinf(smallestAngleBetween(ls.getLineDirection(), moveDir));
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

    // Serial.println(bearing.orientation.x);
    // correction = -bearingCorrection.update(heading, 0.0);
    
    if(motorSwitch && commEnable) {
        if(abs(correction) < 20) {
            motors.run(moveSpeed, moveDir, 0);
        } else{ 
            motors.run(moveSpeed, moveDir, correction);
        }
    } else {
        motors.run(0, 0, 0);
    };
    
    // bool goForward = true;

    // if (testMot.timeHasPassedNoUpdate()) {
    //     goForward = !goForward; // Toggle direction
    // }

    // if (goForward) {
    //     motors.run(100, 0, 0); // Forward
    // } else {
    //     motors.run(100, 180, 0); // Backward
    // }

}