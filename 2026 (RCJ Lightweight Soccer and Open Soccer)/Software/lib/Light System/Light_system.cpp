#include <Light_System.h>

void LightSystem::init() {
    for(uint8_t i = 0; i < 4; i++) {
        pinMode(pinList[i], OUTPUT);
    }
    pinMode(LIGHT_PIN, INPUT);
    pinMode(LIGHT_PIN2, INPUT);

    for(int i = 0; i < NUM_LS; i++) {
        whiteThreshold[i] = read_one(i) + LS_CLB_THRESH;
    }
}

/*!
*
* @brief Reads one sensor : intended for the inner circle
*
* @param sensorNum The sensors index in the pinList array
*
* @return Outputs the reading of provided sensor (0 or 1)
*
*/
int LightSystem::read_one(int sensorNum) {
    if(sensorNum < 0 || sensorNum > NUM_LS) {
        throw 112;
    } else {
         for(int8_t i = 0; i < 4; i++) {
            digitalWrite(pinList[i], (sensorNum>>i)&0x01);
        }
        if(((sensorNum>>4)&0x01) == 0) {
            return analogRead(LIGHT_PIN);
        } else {
            return analogRead(LIGHT_PIN2);
        }
    }
}


/*!
* 
* @brief Function for inner circle direction calculation
* @param rot Robots current rotation
* @param motorOn Whether the motors are on
*/
void LightSystem::inner_circle_direction_calc(float rot, bool motorOn) {
    /*REFS*/
    bool (&sIW)[NUM_LS] = sensorIsWhite;
    int (&sT)[NUM_LS] = whiteThreshold;
    for (int8_t i = 0; i < NUM_LS; i++) {
        sIW[i] = (read_one(i) >= sT[i]);
    }
    if(sIW[0]) {sIW[31] = 1; sIW[1] = 1;}
    for(int8_t i = 0; i < NUM_LS; i++) {
        if(sIW[intMod(i+1, NUM_LS)] && sIW[intMod(i-1, NUM_LS)]) {
            sIW[i] = 1;
        }
    }
    int8_t clustersList[4][2]; float clusterCenter[3] = {44};
    int clusterAmount = 0; int minIndex = 0; int maxIndex = 0;
    for(int8_t i = 0; i < NUM_LS; i++) {
        if(sIW[i]) {
            if(!sIW[intMod(i - 1, NUM_LS)]) {
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
        linePos = calculate_distance_over(clustersList[0][0]*11.25, clustersList[0][1]*11.25);
    } else if(clusterAmount == 2) {
        lineDirection = angleBetween(clusterCenter[0], clusterCenter[1]) <= 180\
                        ? midAngleBetween(clusterCenter[0], clusterCenter[1]) :\
                        midAngleBetween(clusterCenter[1], clusterCenter[0]);
        linePos = calculate_distance_over(clusterCenter[0], clusterCenter[1]);
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
                linePos = calculate_distance_over(clusterCenter[(i+1) % 3], clusterCenter[i]);
                break;
            }
        }
    }
    calculate_line_state(rot, lineDirection, linePos);
    if(!motorOn) {
        lineDir = -1;
        lineState = 0.0;
    }
}

/*!
 * @brief Calculates the state at which the robot is relative to the line and
 *        field.
 * 
 * @param rot Rotation of the robot relative to forward (bno value).
 * @param lineDirection Direction of the line.
 * @param linePos How far over the robot is over the line.
 */
void LightSystem::calculate_line_state(float rot, float lineDirection, float linePos) {
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
 * 
 * @returns A float between 0 and 1 representing the angular difference.
 */
float LightSystem::calculate_distance_over(float angle1, float angle2) {
    float condition = 0.5 * (1 - cosf(DEG_TO_RAD * smallestAngleBetween(angle1, angle2)));
    return (condition == 0)?condition+=0.001:condition;
}

float LightSystem::get_line_dir(){
    return lineDir;
}