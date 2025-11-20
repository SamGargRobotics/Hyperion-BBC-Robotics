#include <LightSystem.h>

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
         for(u_int8_t i = 0; i < 4; i++) {
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
    for (u_int8_t i = 0; i < NUM_LS; i++) {
        sensorIsWhite[i] = (read_one(i) >= whiteThreshold[i]) ?  true : false ;
    }
}