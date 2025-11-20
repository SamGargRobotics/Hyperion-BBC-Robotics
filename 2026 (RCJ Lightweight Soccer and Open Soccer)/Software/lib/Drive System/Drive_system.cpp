#include "Drive_system.h"
#include <math.h>

/*!
 * @brief Initializes the motors for use.
 */
void DriveSystem::init() {
    for(uint8_t i = 0; i < MOTOR_NUM; i++) {
        pinMode(motorInA[i], OUTPUT);
        pinMode(motorInB[i], OUTPUT);
        pinMode(motorPWM[i], OUTPUT);
    }
}

/*!
 * @brief Calculates the motor spd and writes to each motor accordingly.
 * 
 * @param spd spd robot is running at.
 * @param ang ang that the robot should move in.
 * @param cor Rotation needed to ensure that the robot stays forward.
 */
void DriveSystem::run(float spd, float ang, float cor) {
    for(int i = 0; i < MOTOR_NUM; i++) {
        values[i] = cosf(DEG_TO_RAD * (ang + motorAng[i])) * spd + cor;
    }

    float largestSpd = 0.0f;

    for(int i = 0; i < MOTOR_NUM; i++) {
        if(fabs(values[i]) > largestSpd) {
            largestSpd = fabs(values[i]);
        }
    }

    if(largestSpd > 255.0f) {
        for(uint8_t i = 0; i < MOTOR_NUM; i++) {
            values[i] *= (255.0f / largestSpd);
        }
    }
    for(uint8_t i = 0; i < MOTOR_NUM; i++) {
        int finalSpd = round(abs(values[i]));
        analogWrite(motorPWM[i], finalSpd);
        digitalWrite(motorInA[i], (values[i] > 0));
        digitalWrite(motorInB[i], (values[i] < 0));
    }
}