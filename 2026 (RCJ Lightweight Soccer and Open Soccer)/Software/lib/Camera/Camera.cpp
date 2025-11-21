#include "Camera.h"

void Camera::init() {
    cameraSerial.begin(115200);
}

void Camera::update(bool attackBlue) {
    if (cameraSerial.available() < CAM_PACKET_SIZE) return;
    int b1 = cameraSerial.read();
    int b2 = cameraSerial.peek();
    if (b1 != CAM_START_PACK_1 || b2 != CAM_START_PACK_2) return;
    cameraSerial.read();
    int goalXY = cameraSerial.read();
    int goalYY = cameraSerial.read();
    int goalXB = cameraSerial.read();
    int goalYB = cameraSerial.read();
    int ballX  = cameraSerial.read();
    int ballY  = cameraSerial.read();
    shift(goalXY, goalYY);
    shift(goalXB, goalYB);
    shift(ballX,  ballY);
    if (attackBlue) {
        attackGoal._angle  = calc_angle(goalYB, goalXB);
        attackGoal._dist   = calc_distance(goalXB, goalYB);
        attackGoal._visible = (goalYB != 0);

        defendGoal._angle  = calc_angle(goalYY, goalXY);
        defendGoal._dist   = calc_distance(goalXY, goalYY);
        defendGoal._visible = (goalYY != 0);
    }
    else {
        attackGoal._angle  = calc_angle(goalYY, goalXY);
        attackGoal._dist   = calc_distance(goalXY, goalYY);
        attackGoal._visible = (goalYY != 0);

        defendGoal._angle  = calc_angle(goalYB, goalXB);
        defendGoal._dist   = calc_distance(goalXB, goalYB);
        defendGoal._visible = (goalYB != 0);
    }

    ballInfo._angle = calc_angle(ballY, ballX);
    ballInfo._dist  = calc_distance(ballX, ballY);
}

void Camera::shift(int& x, int& y) {
    if (x != 0) {
        x -= 60;
        y -= 60;
    }
}

float Camera::calc_distance(float x, float y) {
    return sqrtf(x*x + y*y);
}

float Camera::calc_angle(float opp, float adj) {
    return fmod(90 - (atan2f(opp, adj) * RAD_TO_DEG), 360);
}