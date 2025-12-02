#include <TSSP_system.h>

void TsspSystem::init() {
    for (int i = 0; i < TSSP_NUM; i++) {
        pinMode(tsspPins[i], INPUT);
    }
}

void TsspSystem::update() {
    ballStr = 0;
    uint8_t tsspValues[TSSP_NUM] = {0};
     uint8_t tsspSortedValues[TSSP_NUM] = {0};
    uint8_t tsspSortedIndex[TSSP_NUM] = {0}; 

    for (int i = 0; i < 255; i++) {
        for (int j = 0; j < TSSP_NUM; j++) {
            tsspValues[j] += 1 - digitalRead(tsspPins[j]);
            delayMicroseconds(1);
        }
    }

    int maxIndex = 0;
    for (int i = 1; i < TSSP_NUM; i++) {
        if (tsspValues[i] >= tsspValues[maxIndex]) {
            maxIndex = i;
        }
    }

    for(uint8_t i = 0; i < TSSP_NUM; i++) {
        for(uint8_t j = 0; j < TSSP_NUM; j++) {
            if(tsspValues[i] > tsspSortedValues[j]) {
                if(j <= i) {
                    ARRAYSHIFTDOWN(tsspSortedValues, j, i);
                    ARRAYSHIFTDOWN(tsspSortedIndex, j, i);
                }
                tsspSortedValues[j] = tsspValues[i];
                tsspSortedIndex[j] = i;
                break;
            }
        }
    }

    float x = ((tsspSortedValues[0] * tsspX[tsspSortedIndex[0]]) + \
              (tsspSortedValues[1] * tsspX[tsspSortedIndex[1]]) + \
              (tsspSortedValues[2] * tsspX[tsspSortedIndex[2]]) + \
              (tsspSortedValues[3] * tsspX[tsspSortedIndex[3]])) / 4;
    float y = ((tsspSortedValues[0] * tsspY[tsspSortedIndex[0]]) + \
              (tsspSortedValues[1] * tsspY[tsspSortedIndex[1]]) + \
              (tsspSortedValues[2] * tsspY[tsspSortedIndex[2]]) + \
              (tsspSortedValues[3] * tsspY[tsspSortedIndex[3]])) / 4;
    
    ballStr = ((3 * tsspSortedValues[0]) + (2 * tsspSortedValues[1]) + 
                        tsspSortedValues[2] + tsspSortedValues[3]) / 7.0;
    ballDir = (ballStr != 0) ? 360 - \
                            floatMod((RAD_TO_DEG * (atan2f(y, x)))-90, 360) : 0;
}
