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

void LightSystem::inner_circle_direction_calc() {
    for (int8_t i = 0; i < NUM_LS; i++) {
        sensorIsWhite[i] = (read_one(i) >= whiteThreshold[i]);
    }
    if(sensorIsWhite[0]) {sensorIsWhite[31] = 1; sensorIsWhite[1] = 1;}
    for(int8_t i = 0; i < NUM_LS; i++) {
        if(sensorIsWhite[intMod(i+1, NUM_LS)] && sensorIsWhite[intMod(i-1, NUM_LS)]) {
            sensorIsWhite[i] = 1;
        }
    }
    int8_t clustersList[4][2]; float clusterCenter[3] = {44};
    int8_t clusterAmt = 0; int8_t minIndex; int8_t maxIndex;
    for(int8_t i = 0; i < NUM_LS; i++) {
        if(sensorIsWhite[i]) {
            if(!sensorIsWhite[intMod(i - 1, NUM_LS)]) {
                minIndex = i;
            }
            if(!sensorIsWhite[intMod(i + 1, NUM_LS)]) {
                maxIndex = i;
                clustersList[clusterAmt][0] = minIndex;
                clustersList[clusterAmt][1] = maxIndex;
                clusterAmt++;
            }  
        }
    }
}