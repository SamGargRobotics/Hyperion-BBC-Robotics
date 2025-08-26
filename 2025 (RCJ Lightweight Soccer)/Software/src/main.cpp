#include <Drive_system.h>
#include <TSSP_system.h>
#include <Light_system.h>
#include <PID.h>
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
PID defenderVert(KP_DEFEND_VERT, KI_DEFEND_VERT, KD_DEFEND_VERT);
PID defenderHozt(KP_DEFEND_HOZT, KI_DEFEND_HOZT, KD_DEFEND_HOZT, 100.0);
PID avoidLine(KP_LINE_AVOID, KI_LINE_AVOID, KD_LINE_AVOID, 255.0);
PID centeringPID(KP_CENTERING, KI_CENTERING, KD_CENTERING);
Timer batteryTimer(5000000);    
Tssp_system tssp;
VoltDiv battery(BATT_READ_PIN, BATTERY1_DIVIDER);
Timer testMot(1000000);
sensors_event_t bearing;

void setup() {
    Serial.begin(9600);
    tssp.init();
    ls.init();
    motors.init();
    bt.init();
    battery.init();
    batteryTimer.resetTime();
    testMot.resetTime();
    cam.init();
    pinMode(GOAL_PIN, INPUT);
    pinMode(COM_PIN, INPUT);
    while(!bno.begin(OPERATION_MODE_IMUPLUS)) {
        Serial.println("No BNO055 detected. Check your wiring or I2C ADDR.");
        delay(1000);
    }
    delay(500);
    bno.setExtCrystalUse(true);
    delay(500);
}

void loop() {
    bool role = bt.getRole();
    tssp.update(role, cam.getAttackGoalDist());
    bt.update(tssp.getBallDir(), tssp.getBallStr(), true);
    bno.getEvent(&bearing);
    cam.update(false, bearing.orientation.x);
    ls.update(bearing.orientation.x, true);
    float moveDir = 0.0;
    float moveSpeed = 0.0;
    float correction = 0.0;
    if((battery.update() <= BATTERY_CRITICAL) && !COMPETITION_MODE) {
        if(batteryTimer.timeHasPassedNoUpdate()) {
            moveSpeed = 0;
            correction = 20;
        }
    } else {
        batteryTimer.resetTime();
        if(ls.getLineState() > ATK_LINE_SP) {
            moveDir = floatMod(ls.getLineDirection() + 180.0, 360.0);
            if(smallestAngleBetween(moveDir, ls.getLineDirection()) < 90) {
                moveSpeed = -avoidLine.update(ls.getLineState(), ATK_LINE_SP);
            } else {
                moveSpeed = BASE_SPEED;
            }
        } else {
            moveDir = tssp.getMoveDir();
            moveSpeed = tssp.getMoveSpd();
            correction = role?cam.getAttackCorrection():cam.getDefendCorrection(); 
        }
    }

    motors.run(moveSpeed, moveDir, correction);
}