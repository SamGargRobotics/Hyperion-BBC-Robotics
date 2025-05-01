#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
    float kp;
    float ki;
    float kd;
    PID(float p, float i, float d, float absoluteMax = 0.0);
    float update(float input, float setpoint, float modulus = 0.0);

private:
    unsigned long lastTime;

    float absMax;

    float integral;
    float lastInput;
};

#endif