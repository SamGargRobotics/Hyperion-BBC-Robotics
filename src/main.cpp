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
    float correction = 0.0;

    if(bt.getRole()) {
        if(tssp.getBallStr() != 0) {
            float modBallDir = tssp.getBallDir() > 180 ? tssp.getBallDir() - 360
                               : tssp.getBallDir();
            float moveScaler = constrain(tssp.getBallStr() / ORBIT_STRENGTH_RADIUS, 0, 1);
            float moveOffset = moveScaler * 0.4 * expf(0.25 * abs(modBallDir)) - 0.4; 
            moveDir = floatMod(moveOffset + tssp.getBallDir(), 360.0);
            moveSpeed = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0 - moveOffset / 90.0);
            if(cam.getAttackGoalVisible()) {
                float goalHeading = cam.getAttackGoalAngle() > 180 ? cam.getAttackGoalAngle() - 360 : cam.getAttackGoalAngle();
                correction = -camAttackCorrection.update(goalHeading, 0);
            } else {
                correction = -bearingCorrection.update(heading, 0);
            }
        } else {
            moveDir = (bearing.orientation.x < 360 && bearing.orientation.x > 180)? 90 : 270;
            moveSpeed = centeringPID.update(heading, 0);
        }
        if(ls.getLineState() > 0.0) {
            if(smallestAngleBetween(ls.getLineDirection(), moveDir) < 60.0 || ls.getLineState() > LINE_STRICT_AVOID) {
                moveDir = floatMod(ls.getLineDirection() + 180, 360.0);
                moveSpeed = -avoidLine.update(ls.getLineState(), ATK_LINE_SP);
            }
        }
    } else {
        if(tssp.getBallDir > 110 && tssp.getBallDir <= 290) {
            moveDir = 180;
            moveSpeed = BASE_SPEED;
        } else {
            float vertVect = -defenderVert.update(ls.getLineState(), DEF_VERT_SP);
            float defHeading = (tssp.getBallStr != 0)?((tssp.getBallDir() > 180)?tssp.getBallDir() - 360 : tssp.getBallDir()) : -heading;
            float hoztVect = -defenderHozt.update(defHeading, 0);
            moveDir = floatMod(atan2f(hoztVect, vertVect)*RAD_TO_DEG, 360);
            moveSpeed = sqrtf(powf(vertVect, 2) + powf(hoztVect, 2));
        }
        if(cam.getDefendGoalVisible()) {
            float goalHeading = cam.getDefendGoalAngle() > 180 ? cam.getDefendGoalAngle() - 360 : cam.getDefendGoalAngle();
            correction = -camDefendCorrection(goalHeading, 0);
        } else {
            correction = -bearingCorrection.update(heading, 0);
        }
        if(ls.getLineState() > 0.0) {
            if(smallestAngleBetween(ls.getLineDirection(), moveDir) < 60.0 && ls.getLineState() > LINE_STRICT_AVOID) {
                moveDir = floatMod(ls.getLineDirection() + 180, 360.0);
                moveSpeed = -avoidLine.update(ls.getLineState(), DEF_LINE_SP);
            }
        }
    }
    motors.run(moveSpeed, moveDir, correction);
}





/*







    float goalAngle = 0;
    float goalDistance = 0;
    bool goalVisible = 0;
    if(attacking) {
        goalAngle = cam.getAttackGoalAngle();
        goalDistance = cam.getAttackGoalDist();
        goalVisible = cam.getAttackGoalVisible();
    } else {
        goalAngle = cam.getDefendGoalAngle();
        goalDistance = cam.getDefendGoalDist();
        goalVisible = cam.getDefendGoalVisible();
    }
    float goalHeading = (attacking)? (goalAngle < 180 ? \
                                floatMod(-goalAngle, 360) - 360 : \
                                floatMod(-goalAngle, 360))
                                
                                
                                : \
                                floatMod(-goalAngle, 360);
    float camDefendCorretion = goalVisible?camDefendCorrection.update(
                               goalHeading, 180):rotation;
    
    ls.calculateLineDirection();
    float lineState = ls.getLineState();
    float lsMoveAngle = (ls.getLineDirection()== -1)? -1 : floatMod((ls.getLineDirection() - bearing.orientation.x) \
                        + 180, 360);

    float batLevel = battery.update();

    float attackMoveDir = dirCalc.exponentialOrbit(tssp.getBallDir());
    float attackMoveSpd = dirCalc.calcSpeed(tssp.getBallStr(), tssp.getBallDir())*SET_SPEED;
    attackMoveSpd = max(50.0f, ((goalAngle >= 5 || goalAngle <= -15) ||
                        !goalVisible)? \
                        attackMoveSpd/4 : attackMoveSpd);
    
    float defVertStPt = (tssp.getBallStr() != 0) ? ((tssp.getBallDir() > 180) ? (tssp.getBallDir() - 360) : \
                        tssp.getBallDir()) : -(bearing.orientation.x>180) \
                        ?(bearing.orientation.x-360) : bearing.orientation.x;
    float vertDefMovement = -defenderMovementVert.update(lineState, \
                            DEF_LINE_SP);
    float hoztDefMovement = -defenderMovementHozt.update(defVertStPt, 0);
    float netDefMoveAngle = floatMod(atan2(hoztDefMovement, vertDefMovement)\
                            *RAD_TO_DEG, 360);
    float netDefSpd = sqrt(pow(vertDefMovement, 2) + pow(hoztDefMovement, 2));
}

// // [Main Global Vars]
// //! @brief Y value of the chosen goal that is being tracked.
// uint8_t goal_y_val = 0;
// //! @brief X value of the chosen goal that is being tracked.
// uint8_t goal_x_val = 0;
// //! @brief The amount the robot must correct to stay forward in terms of camera
// float goalHeading = 0;
// //! @brief The amount the robot must correct to stay forward in terms of compass
// float bnoHeading = 0;
// //! @brief The amount of correction in terms of heading for the bno
// float bnoCorrection = 0;
// //! @brief The amount of correction in terms of heading for the camera (attack)
// float cameraAttackCorrection = 0;
// //! @brief The amount of correction in terms of heading for the camera (defend)
// float cameraDefenceCorrection = 0;
// //! @brief Angle to goal of the chosen goal that is being tracked.
// float goal_angle = 0;
// //! @brief Distance of goal to robot (horizontal - pixels)
// float goal_dis = 0;
// //! @brief Goal pixel size.
// float goal_pixel = 0;
// //! @brief Current Compass Value (rotation)
// float rot = 0;
// //! @brief The orbit values of the attacking function
// float attackerMoveDirection = 0;
// //! @brief The movement speed of the robot based on function (attack)
// float attackerMoveSpeed = 0;
// //! @brief The movement based on the veritcal PID of the robot (defend)
// float verticalDefenderMovement = 0;
// //! @brief The movement based on the horizontal PID of the robot (defend) 
// float horizontalDefenderMovement = 0;
// //! @brief Net Defender Movement Angle with Hozt and Vert calcs 
// float netDefendMovementAngle = 0;
// //! @brief Net Defender Speed with HOzt and Vert calc
// float netDefendSpeed = 0;
// //! @brief Current battery level of the robot (amps)
// float batteryCurrentLevel = 0;
// //! @brief Current battery level of the robot (volts)
// float currentBatteryLevelVolts = 0;
// //! @brief Move Angle in relevancy to the ls.lineDirection
// float lsMoveAngle = -1;
// //! @brief Vertical heading of the defender calculations (vectors)
// float defenderVertHeading = 0;
// //! @brief Assigned move speed of the robot throughout the logic
// float _moveSpeed = 0;
// //! @brief Assigned move angle of the robot throughout the logic
// float _moveAngle = 0;
// //! @brief Assigned move rotation of the robot throughout the logic
// float _moveRotation = 0;
// //! @brief Goal that is chosen to be tracker (attacker wise)
// bool chosenGoal = targetGoal;
// //! @brief Current logic state of the robot
// String robotState = "default";
// //! @brief State that the correction is in
// String correctionState = "default";

// void loop() {
// // [Logic Pin Calculations]
//     chosenGoal = digitalRead(LOGIC_PIN);

// // [Tssp Calculations]
//     tssp.update();
//     #if DEBUG_TSSP
//         Serial.print(tssp.tssp.getBallDir());
//         Serial.print("\t");
//         Serial.print(tssp.detectingBall);
//         Serial.print("\t");
//         Serial.println(tssp.tssp.getBallStr());
//     #endif

// // [Bluetooth]
//     dirCalc.attack = bluetooth.update(dirCalc.attack, tssp.tssp.getBallDir(), 
//                      tssp.tssp.getBallStr());
//     dirCalc.attack = 0;
    
// // [Correction / Goal Tracking Calculations]
//     compass.getEvent(&rotation);
//     rot = rotation.orientation.x;
//     cam.read_camera();
    
//     // Decide which goal is being tracked
//     if(chosenGoal) {
//         goal_y_val = (dirCalc.attack)? cam.goal_y_blue : cam.goal_y_yellow;
//         goal_x_val = (dirCalc.attack)? cam.goal_x_blue : cam.goal_x_yellow;
//         goal_angle = (dirCalc.attack)? cam.angle_to_goal_blue : \
//                                        cam.angle_to_goal_yellow;
//         goal_pixel = cam.distBlue;
//     } else {
//         goal_y_val = (dirCalc.attack)? cam.goal_y_yellow : cam.goal_y_blue;
//         goal_x_val = (dirCalc.attack)? cam.goal_x_yellow : cam.goal_x_blue;
//         goal_angle = (dirCalc.attack)? cam.angle_to_goal_yellow : \
//                                        cam.angle_to_goal_blue;
//         goal_pixel = cam.distYel;
//     }
//     // Complete floatMod values to ensure that the heading is not constantly 
//     // changing when the robot faces the goal.
//     goalHeading = (dirCalc.attack)? (floatMod(-goal_angle, 360) > 180 ? \
//                                     floatMod(-goal_angle, 360) - 360 : \
//                                     floatMod(-goal_angle, 360)) : \
//                                     floatMod(-goal_angle, 360);
//     // Regular correction using the BNO/IMU/Compass
//     bnoHeading = (rot>180)?(rot-360):rot;
//     // Assign PID's variables
//     bnoCorrection = -regularCorrection.update(bnoHeading, 0);
//     cameraAttackCorrection = -goalAttackingCorrection.update(goalHeading, 0);
//     cameraDefenceCorrection = -goalDefendingCorrection.update(goalHeading, 180);
//     // Heading logic to assign goal tracking or regular compass correct
//     // Calculate distance of the goals away from the robot (pixels)
//     goal_dis = (sqrt(pow(abs(goal_x_val), 2) + pow(abs(goal_y_val), 2)));
//     // // Hard coded offset value as the camera was reading different values on
//     // // each side.
//     goal_dis = (goalHeading < 180)?(goal_dis + GOAL_DIS_OFFSET) : goal_dis;
//     #if not GOAL_TRACKING_TOGGLE
//         cameraAttackCorrection = bnoCorrection;
//         cameraDefenceCorrection = bnoCorrection;
//     #endif
//     #if DEBUG_IMU_CAM
//         Serial.print(rot);
//         Serial.print("\t");
//         Serial.print(bnoHeading);
//         Serial.print("\t");
//         Serial.print(bnoCorrection);
//         Serial.print("\t");
//         Serial.print(goalHeading);
//         Serial.print("\t");
//         Serial.print(cam.goal_x_blue);
//         Serial.print("\t");
//         Serial.print(cam.goal_y_blue);
//         Serial.print("\t"); 
//         Serial.print(cam.angle_to_goal_blue);
//         Serial.print("\t");
//         Serial.print(cam.goal_x_yellow);
//         Serial.print("\t");
//         Serial.print(cam.goal_y_yellow);
//         Serial.print("\t"); 
//         Serial.print(cam.angle_to_goal_yellow);
//         Serial.print("\t");
//         Serial.println(goal_dis);
//     #endif

// // [Battery Level Calculations]
//     batteryLevel.read();

//     #if BAT_READ_VOLTS
//         if(batteryLevel.motorOn) {
//             Serial.print("MOTOR ON");
//         } else {
//             Serial.print("MOTOR OFF");
//         }
//         Serial.print("\t");
//         Serial.println(batteryLevel.volts);
//     #endif

// // [Light Sensors]
//     ls.calculateLineDirection(rot);
//     #if DEBUG_LS
//         Serial.print("ClusterAmt: ");
//         Serial.print(ls.clusterAmount);
//         Serial.print("\t"); 
//         Serial.print("LineState: ");
//         Serial.print(ls.lineState);
//         Serial.print("\t");
//         Serial.print("Move: ");
//         Serial.print(lsMoveAngle);
//         Serial.print("\t");
//         Serial.print("Line: ");
//         Serial.println(ls.lineDir);
//     #endif

// // [Strategy and Movement Calculation]
//     attackerMoveDirection = dirCalc.exponentialOrbit(tssp.tssp.getBallDir());  
//     attackerMoveSpeed = dirCalc.calcSpeed(tssp.tssp.getBallStr(), tssp.tssp.getBallDir())*SET_SPEED;
//     attackerMoveSpeed = max(50.0f, ((goal_angle >= 5 || goal_angle <= -15) || \
//                         (goal_x_val == 0 && goal_y_val == 0))? \
//                         attackerMoveSpeed/4 : attackerMoveSpeed);
//     defenderVertHeading = (tssp.detectingBall) ? ((tssp.tssp.getBallDir() > 180) ?
//                           (tssp.tssp.getBallDir() - 360) : tssp.tssp.getBallDir()) :
//                           -bnoHeading;
//     // defenderVertHeading = (tssp.tssp.getBallDir() > 180) ? (tssp.tssp.getBallDir() - 360) : (tssp.tssp.getBallDir());
//     verticalDefenderMovement = -defenderMovementVert.update(ls.lineState, \
//                                                 GOAL_SEMI_CIRCLE_RADIUS_CM);
//     Serial.print(ls.lineState); Serial.print("\t");
//     Serial.print(verticalDefenderMovement); Serial.print("\t");
//     horizontalDefenderMovement = -defenderMovementHozt.update(
//                                   defenderVertHeading, 0);
//     netDefendMovementAngle = floatMod(atan2(horizontalDefenderMovement, \
//                              verticalDefenderMovement)*RAD_TO_DEG, 360);
//     Serial.print(netDefendMovementAngle); Serial.print("\t");
//     netDefendSpeed = sqrt(pow(verticalDefenderMovement, 2) + \
//                          pow(horizontalDefenderMovement, 2));
//     Serial.println(netDefendSpeed);
//     if ((tssp.tssp.getBallDir() >= 350 || tssp.tssp.getBallDir() <= 10) && \
//         (tssp.tssp.getBallStr() >= DEFENCE_SURGE_STR_VALUE) && !surgestates.surgeQ) {
//             surgestates.surgeQ = true;
//             surgestates.startMillis = micros();
//         }
//     if((tssp.tssp.getBallStr() <= 60) || (surgestates.startMillis+3000000) <= micros() ||\
//         ((tssp.tssp.getBallDir() >= 10 && tssp.tssp.getBallDir() <= 350))) {
//         surgestates.surgeQ = false;
//     }

// // [Moving the Robot Final Calculations and Logic]
//     bool ignore_line_avoid = 0;
//     surgestates.surgeQ = false;
//     #if DEBUG_ROBOT
//         motors.run(0, 0, bnoCorrection);
//     #else
//         if(dirCalc.attack) {
//             // If robot is attacking --> Attacker Logic
//             if((tssp.tssp.getBallDir() >= 347.5 || tssp.tssp.getBallDir() <= 12.5) && \
//                 tssp.tssp.getBallStr() >= SURGE_STR_VALUE) {
//                 // If ball is generally straight and in capture --> surge
//                 _moveSpeed = tssp.detectingBall?SET_SPEED:0;
//                 _moveAngle = goal_angle;
//                 _moveRotation = cameraAttackCorrection;
//                 _moveRotation += random(-10,10);
//                 robotState = "Attacker Logic - Surge";
//             } else {
//                 if(tssp.tssp.getBallStr() > ORBIT_STRENGTH_RADIUS) {
//                     // If ball is close then orbit
//                     _moveSpeed = tssp.detectingBall?attackerMoveSpeed:0;
//                     _moveAngle = attackerMoveDirection;
//                     _moveRotation = cameraAttackCorrection;
//                     correctionState = "Regular";
//                     robotState = "Attacker Logic - Orbit";
//                 } else {
//                     // If ball is far then ball follow
//                     _moveSpeed = tssp.detectingBall?attackerMoveSpeed:0;
//                     _moveAngle = tssp.tssp.getBallDir();
//                     _moveRotation = cameraAttackCorrection;
//                     robotState = "Attacker Logic - Ball Follow";
//                 }
//             }
//         } else {
//             // Defender Logic
//             if(surgestates.surgeQ) {
//                 _moveSpeed = SET_SPEED;
//                 _moveAngle = tssp.tssp.getBallDir();
//                 _moveRotation = cameraAttackCorrection;
//                 correctionState = "Goal";
//                 robotState = "Defender Logic - Surge";
//             } else {
//                 // If ball not in capture
//                 if(ls.lineDir == -1) {
//                     // If can see line
//                     if(tssp.tssp.getBallDir() >= 110 && tssp.tssp.getBallDir() <= 290) {
//                         // If ball is in threshold robot --> orbit
//                         _moveSpeed = attackerMoveSpeed;
//                         _moveAngle = attackerMoveDirection;
//                         _moveRotation = bnoCorrection;
//                         correctionState = "Regular";
//                         robotState = "Defender Logic - Ball Behind";
//                     } else {
//                         ignore_line_avoid = true;
//                         _moveSpeed = netDefendSpeed;
//                         _moveAngle = netDefendMovementAngle;
//                         _moveRotation = cameraDefenceCorrection;
//                         robotState = "Defender Logic - Regular";
//                     }
//                 } else {
//                     // If cannot see goal --> Forward or Backward depending
//                     // on last seen goal_y_val
//                     _moveSpeed = SET_SPEED;
//                     _moveAngle = 180;
//                     correctionState = "Regular";
//                     robotState = "Defender Logic - Cannot see line";
//                 }
//             }
//         }
//         if((ls.lineDir != -1) && !ignore_line_avoid) {
//             // If detecting line --> Line Avoidance
//             _moveAngle = lsMoveAngle;
//             _moveSpeed = SET_SPEED;
//             robotState = "Line Avoidance";
//         }
//         motors.run(_moveSpeed, _moveAngle, _moveRotation);
//     #endif
//     #if DEBUG_ROBOT_STATE
//         Serial.print(robotState);
//         Serial.print("\t");
//         Serial.print(correctionState);
//         Serial.print("\t Detecting Ball?: ");
//         Serial.println(tssp.detectingBall);
//     #endif
//     // if battery level is less than battery critical level, spin in spot
//     #if not COMPETITION_MODE
//         if(batteryLevel.volts <= BATTERY_CRITICAL) {
//             if(batteryTimer.timeHasPassedNoUpdate()) {
//                 motors.run(0,0,20);
//             }
//         } else {
//             batteryTimer.resetTime();
//         }
//     #endif

// // [Manual Printing Space]
// }
*/