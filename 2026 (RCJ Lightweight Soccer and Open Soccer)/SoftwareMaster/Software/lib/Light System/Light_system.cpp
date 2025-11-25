#include <Light_System.h>

/*!
 * @brief Initializes the light sensor system.
 *
 * Sets the pin modes for all sensor control pins and input pins.
 * Loads previously stored sensor thresholds from EEPROM, or sets
 * default values if EEPROM values are invalid or uninitialized.
 *
 * @note The function reads EEPROM values from @ref EEPROM_START_ADDR.
 *       If a value is invalid or uninitialized (EEPROM_CLEAR_VALUE), 
 *       a default threshold of 200 is used.
 */
void LightSystem::init() {
    for(uint8_t i = 0; i < 4; i++) {
        pinMode(pinList[i], OUTPUT);
    }
    pinMode(LIGHT_PIN, INPUT);
    pinMode(LIGHT_PIN2, INPUT);

    for(int i = 0; i < NUM_LS; i++) {
        int val = EEPROM.read(EEPROM_START_ADDR + i);
        // If EEPROM was never written, fallback to a default value
        if(val == EEPROM_CLEAR_VALUE || val > 255 || val < 0) {
            whiteThreshold[i] = 200; // some reasonable default
        } else {
            whiteThreshold[i] = val;
        }
    }
}

/*!
 * @brief Calibrates all light sensors to detect white lines.
 *
 * Performs a sensor sweep and determines if each sensor detects a white line.
 * Updates the `whiteThreshold` array accordingly:
 *   - If a sensor sees white, threshold is set below the reading minus LS_OFFSET.
 *   - If no white is detected, threshold is set above the reading plus BACKUP_OFFSET.
 * The calibrated thresholds are then saved to EEPROM for persistent storage.
 *
 * @note EEPROM is cleared before writing new thresholds.
 * @note The function uses constants CALIBRATE_THRESHOLD, LS_OFFSET, and BACKUP_OFFSET.
 */
void LightSystem::calibrate() {
    int readings[NUM_LS];
    bool detected[NUM_LS] = {false};
    // First, read all sensors and check for white line
    for(int i = 0; i < NUM_LS; i++) {
        readings[i] = read_one(i);
        if(read_one(i) >= CALIBRATE_THRESHOLD) {
            detected[i] = true;  // Sensor sees white line
        }
    }
    // Clear EEPROM first
    for(int i = 0; i < NUM_LS; i++) {
        EEPROM.write(EEPROM_START_ADDR + i, EEPROM_CLEAR_VALUE);
    }
    // Write thresholds
    for(int i = 0; i < NUM_LS; i++) {
        if(detected[i]) {
            whiteThreshold[i] = readings[i] - LS_OFFSET;  // Normal calibrated value
        } else {
            whiteThreshold[i] = readings[i] + BACKUP_OFFSET;  // Backup threshold
        }
        // Save to EEPROM
        EEPROM.write(EEPROM_START_ADDR + i, whiteThreshold[i]);
    }
}

/*!
* @brief Reads one sensor : intended for the inner circle
* @param sensorNum The sensors index in the pinList array
* @return Outputs the reading of provided sensor (0 or 1)
*/
int LightSystem::read_one(int sensorNum) {
    if(sensorNum < 0 || sensorNum > NUM_LS) {
        throw 112;
    } else {
         for(int8_t i = 0; i < 4; i++) {
            digitalWrite(innerPinList[i], (sensorNum>>i)&0x01);
        }
        if(((sensorNum>>4)&0x01) == 0) {
            return analogRead(LIGHT_PIN);
        } else {
            return analogRead(LIGHT_PIN2);
        }
    }
}

/*!
* @brief Reads one sensor : intended for the outer circle
* @param sensorNum The sensors index in the outerPinList array
* @return Outputs the reading of provided sensor (0 or 1)
*/
int LightSystem::read_one_outer(int sensorNum) {
    if(sensorNum < 0 || sensorNum > OUTER_NUM_LS) {
        throw 112;
    } else {
         for(int8_t i = 0; i < 4; i++) {
            digitalWrite(outerPinList[i], (sensorNum>>i)&0x01);
        }
        if(((sensorNum>>4)&0x01) == 0) {
            return analogRead(LIGHT_PIN);
        } else {
            return analogRead(LIGHT_PIN2);
        }
    }
}

/*!
* @brief Function for inner circle direction calculation
* @param rot Robots current rotation
*/
void LightSystem::inner_circle_direction_calc(float rot) {
    /*VARIABLES*/
        int8_t clustersList[4][2]; float clusterCenter[3] = {44};
        int clusterAmount = 0; int minIndex = 0; int maxIndex = 0;
        float lineDirection = -1; float linePos = 0;
    /*AKA's | VARIABLE SHORT HANDS*/
        bool (&sIW)[NUM_LS] = sensorIsWhite; int8_t (&cL)[4][2] = clustersList;
        int (&sT)[NUM_LS] = whiteThreshold; float (&cC)[4][2] = clusterCenter;
        int (&miI) = minIndex; int (&maI) = maxIndex; int (&cA) = clusterAmount;
        float (&lD) = lineDirection; float (&lP) = linePos;
    // Checks Every Sensor To See If They Are White Or !
        for (int8_t i = 0; i < NUM_LS; i++) {
            sIW[i] = (read_one(i) >= sT[i]);
        }
    // Bleed Thee Index Loops Together
        if(sIW[0]) {sIW[31] = 1; sIW[1] = 1;}
    // Fills In The Gaps For Broken Sensors
        for(int8_t i = 0; i < NUM_LS; i++) {
            if(sIW[com.intMod(i+1, NUM_LS)] && sIW[com.intMod(i-1, NUM_LS)]) {
                sIW[i] = 1;
            }
        }
    // Creates The Clusters
        for(int8_t i = 0; i < NUM_LS; i++) {
            if(sIW[i]) {
                if(!sIW[com.intMod(i - 1, NUM_LS)]) {
                    miI = i;
                }
                if(!sIW[com.intMod(i + 1, NUM_LS)]) {
                    maI = i;
                    cL[cA][0] = miI;
                    cL[cA][1] = maI;
                    cA++;
                }  
            }
        }
        if(sIW[31] && sIW[0]) {
            cL[0][0] = miI;
        }
    // Finds The Angle Of Every Cluster
        for(int i = 0; i < cA; i++) {
            cC[i] = com.midAngleBetween(cL[i][0]*11.25, cL[i][1]*11.25);
        }
    // Finds The Direction Of The Line
        if(cA == 1) {
            lD = cC[0]; // just the 1 cluster angle
            lP = calculateDistanceOver(cL[0][0]*11.25, cL[0][1]*11.25);
        } else if(cA == 2) {
            lD = angleBetween(cC[0], cC[1]) <= 180 ? midAngleBetween(cC[0], cC[1]) :\
                            midAngleBetween(cC[1], cC[0]);
            lP = calculateDistanceOver(cC[0], cC[1]);
        } else if(cA == 3) {
            float d[3];
            for(int i = 0; i < 3; i++) {
                d[i] = angleBetween(cC[i], cC[(i + 1) % 3]);
            }
            float bD = max(d[0], max(d[1], d[2]));
            for(int i = 0; i < 3; i++) {
                if(bD == d[i]) {
                    lD = midAngleBetween(cC[(i+1) % 3], cC[i]);
                    lP = calculateDistanceOver(cC[(i+1) % 3], cC[i]);
                    break;
                }
            }
        }
    lineDir = lD;
    calculateLineState(rot, lineDirection, linePos);
}

/*!
 * @brief Calculates the state at which the robot is relative to the line and
 *        field.
 * 
 * @param rot Rotation of the robot relative to forward (bno value).
 * @param lineDirection Direction of the line.
 * @param linePos How far over the robot is over the line.
 */
void Light_system::calculate_line_state(float rot, float lineDirection, float linePos) {
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

void LightSystem::outer_circle_dir_calc() {
    float lineDirection = -1; int isOn[16] = {0}; 
    int (&O) = isOn; int x = 0; int y = 0;
    for(int i = 0; i < 16; i++) {
        isOn[i] = read_one_outer(i);
    }
    if((O[0]+O[1]+O[2]+O[3]) >= 0.4){y++;}
    if((O[4]+O[5]+O[6]+O[7]) >= 0.4){x++;}
    if((O[8]+O[9]+O[10]+O[11]) >= 0.4){y--;}
    if((O[12]+O[13]+O[14]+O[15]) >= 0.4){x--;}
    lineDir = atan2(y, x) * 180.0f / M_PI;
    if (lineDir < 0) lineDir += 360.0f;
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
    float condition = 0.5 * (1 - cosf(DEG_TO_RAD * com.smallestAngleBetween(angle1, angle2)));
    return (condition == 0)?condition+=0.001:condition;
}

float LightSystem::get_line_dir(){
    return lineDir;
}