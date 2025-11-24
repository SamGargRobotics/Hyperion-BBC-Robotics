#include "Camera.h"

/**
 * @brief Initialises the camera communication interface.
 *
 * Opens the high-speed serial link used by the vision module (typically an
 * OpenMV or equivalent) at 115200 baud. This must be called once during system
 * startup before any calls to @ref update.
 *
 * The camera is expected to send structured packets defined by:
 *  - CAM_START_PACK_1
 *  - CAM_START_PACK_2
 *  - CAM_PACKET_SIZE
 *
 * @return void
 */
void Camera::init() {
    cameraSerial.begin(115200);
}

/**
 * Processes a full vision packet and updates goal/ball data.
 *
 * Steps:
 * 1. Validate packet (size + start bytes).
 * 2. Extract raw positions:
 *      - Blue goal (goalXB, goalYB)
 *      - Yellow goal (goalXY, goalYY)
 *      - Ball (ballX, ballY)
 * 3. Shift coordinates into robot space.
 * 4. Apply perspective transforms to get angle + distance.
 * 5. Select attack goal based on `attackBlue`.
 * 6. Mark objects visible when their Y ≠ 0.
 *
 * @param attackBlue  True → attack blue goal; false → attack yellow.
 */
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

/**
 * @brief Shifts raw camera coordinates into the robot’s reference frame.
 *
 * Many vision systems output pixel positions in a top-left-origin format.
 * This method re-centres coordinates so that:
 *  - (0,0) represents camera centre
 *  - +x is rightward
 *  - +y is forward (away from robot)
 *
 * The offset value (60) is determined by camera resolution and calibration.
 *
 * No shift is applied when x == 0 (indicating "no detection").
 *
 * @param x Reference to x-coordinate.
 * @param y Reference to y-coordinate.
 *
 * @return void
 */
void Camera::shift(int& x, int& y) {
    if (x != 0) {
        x -= 60;
        y -= 60;
    }
}

/**
 * @brief Computes Euclidean distance from shifted camera coordinates.
 *
 * @param x X-axis displacement (robot-centric).
 * @param y Y-axis displacement (robot-centric).
 *
 * @return float Estimated distance to the target or ball.
 */
float Camera::calc_distance(float x, float y) {
    return sqrtf(x*x + y*y);
}

/**
 * @brief Computes the angle to a target or ball using atan2 and camera orientation.
 *
 * @param opp Opposite side (usually the y value).
 * @param adj Adjacent side (usually the x value).
 *
 * @return float Angle in degrees, robot-centric.
 */
float Camera::calc_angle(float opp, float adj) {
    return fmod(90 - (atan2f(opp, adj) * RAD_TO_DEG), 360);
}