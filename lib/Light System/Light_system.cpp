/*!
 * @file LSLB.cpp
 *
 * @mainpage LS calculations for Robot
 *
 * This is a library for LS calculations within the robot's code.
 */
 
#include "Light_system.h"
 
/*!
 * @brief Initalizes The Light System, and creates thresholds based on initially
 *        read values.
 */
void Light_system::init() {
    for(uint8_t i = 0; i < 4; i++) {
        pinMode(pinList[i], OUTPUT);
    }
    pinMode(LIGHT_PIN, INPUT);
    pinMode(LIGHT_PIN2, INPUT);

    for(int i = 0; i < 32; i++) {
        whiteThreshold[i] = readOne(i) + 200;
    }
}
 
/*!
 * @brief Reads a singlular light sensor.
 * 
 * @param sensor_num The sensor index in the light sensor array that you want
 *                   to read.
 * 
 * @return Returns the analogue value of the sensor that is read.
 */
int Light_system::readOne(int sensor_num) {
    for(u_int8_t i = 0; i < 4; i++) {
        digitalWrite(pinList[i], (sensor_num>>i)&0x01);
    }
    if(((sensor_num>>4)&0x01) == 0) {
        return analogRead(LIGHT_PIN);
    } else {
        return analogRead(LIGHT_PIN2);
    }
}
 
/*!
 * @brief Calculates the direction of the white line on the field.
 * 
 * @param rot Rotation of the robot (bno value).
 * 
 * @return Calculated line direction with line state taken into account.
 */
void Light_system::update(float rot) {

    for(int i = 0; i < NUM_LS; i++) {
        sensorIsWhite[i] = readOne(i) >= whiteThreshold[i];
    }
    if(sensorIsWhite[0]) {
        sensorIsWhite[31] = 1;
        sensorIsWhite[1] = 1;
    }
    
    for(int i = 0; i < NUM_LS; i++) {
        if(sensorIsWhite[intMod(i+1, NUM_LS)] && sensorIsWhite[intMod(i-1, NUM_LS)]) {
            sensorIsWhite[i] = 1;
        }
    }

    uint8_t clusterAmount = 0;
    for(int i = 0; i < NUM_LS; i++) {
        if(sensorIsWhite[i]) {
            if(!sensorIsWhite[intMod(i - 1, NUM_LS)]) {
                minIndex = i;
            }
            if(!sensorIsWhite[intMod(i + 1, NUM_LS)]) {
                maxIndex = i;
                clustersList[clusterAmount][0] = minIndex;
                clustersList[clusterAmount][1] = maxIndex;
                clusterAmount++;
            }  
        }
    }
    if(sensorIsWhite[31] && sensorIsWhite[0]) {
        clustersList[0][0] = minIndex;
    }

    for(int i = 0; i < clusterAmount; i++) {
        clusterCenter[i] = midAngleBetween(clustersList[i][0]*11.25, clustersList[i][1]*11.25);
    }
    float lineDirection = -1;
    float linePos = 0;
    if(clusterAmount == 1) {
        lineDirection = clusterCenter[0]; // just the 1 cluster angle
        linePos = calculateDistanceOver(clustersList[0][0]*11.25, clustersList[0][1]*11.25);
    } else if(clusterAmount == 2) {
        lineDirection = angleBetween(clusterCenter[0], clusterCenter[1]) <= 180\
                        ? midAngleBetween(clusterCenter[0], clusterCenter[1]) :\
                        midAngleBetween(clusterCenter[1], clusterCenter[0]);
        linePos = calculateDistanceOver(clusterCenter[0], clusterCenter[1]);
    } else if(clusterAmount == 3) {
        float differences[3];
        for(int i = 0; i < 3; i++) {
            differences[i] = angleBetween(clusterCenter[i], clusterCenter[(i + 1) % 3]);
        }
        float bigDiff = max(differences[0], max(differences[1], differences[2]));
        for(int i = 0; i < 3; i++) {
            if(bigDiff == differences[i]) {
                // lineDirection = clusterCenter[(i+2)%3];
                lineDirection = midAngleBetween(clusterCenter[(i+1) % 3], clusterCenter[i]);
                linePos = calculateDistanceOver(clusterCenter[(i+1) % 3], clusterCenter[i]);
                break;
            }
        }
    }
    calculateLineState(rot, lineDirection, linePos);
}

/*!
 * @brief Calculates the state at which the robot is relative to the line and
 *        field.
 * @param rot Rotation of the robot (bno)
 * @param lineDirection Direction of the line
 */
void Light_system::calculateLineState(float rot, float lineDirection, float linePos) {
    bool onLine = lineDirection != -1;
    float relLineDirection = onLine ? floatMod(lineDirection + rot, 360.0) : -1;
    if(lineState == 0.0) {
        if(onLine) {
            lineState = linePos;
            lineDir = relLineDirection;
        }
    } else if(lineState <= 1.0) {
        if(!onLine) {
            lineState = 0.0;
            lineDir = -1.0;
        } else if(smallestAngleBetween(lineDir, relLineDirection) > LS_FLIP_THRESH) {
            lineState = 2 - linePos;
            lineDir = floatMod(relLineDirection + 180.0, 360.0);
        } else {
            lineDir = relLineDirection;
            lineState = linePos;
        }
    } else if(lineState <= 2.0) {
        if(!onLine) {
            lineState = 3;
        } else if(smallestAngleBetween(lineDir, relLineDirection) < (180 - LS_FLIP_THRESH)) {
            lineState = linePos;
            lineDir = relLineDirection;
        } else {
            lineDir = floatMod(relLineDirection + 180.0, 360.0);
            lineState = 2 - linePos;
        }
    } else if(lineState == 3) {
        if(onLine && smallestAngleBetween(lineDir, relLineDirection) > LS_FLIP_THRESH) {
            lineState = 2 - linePos;
            lineDir = floatMod(relLineDirection + 180, 360);
        }
    }
}

/**
 * @brief Calculates a normalized distance metric between two angles.
 *
 * This function computes a value between 0 and 1 that represents the "distance over"
 * or difference between two angles, `angle1` and `angle2`, using a cosine-based smoothing.
 * The result is smallest (0) when the angles are identical and approaches 1 as the
 * angular difference increases.
 *
 * @param angle1 The first angle in degrees.
 * @param angle2 The second angle in degrees.
 * @return A float between 0 and 1 representing the angular difference.
 */
float Light_system::calculateDistanceOver(float angle1, float angle2) {
    float condition = 0.5 * (1 - cosf(DEG_TO_RAD * smallestAngleBetween(angle1, angle2)));
    return (condition == 0)?condition+=0.001:condition;
}

float Light_system::getLineState() {
    return lineState;
}

float Light_system::getLineDirection() {
    return lineDir;
}