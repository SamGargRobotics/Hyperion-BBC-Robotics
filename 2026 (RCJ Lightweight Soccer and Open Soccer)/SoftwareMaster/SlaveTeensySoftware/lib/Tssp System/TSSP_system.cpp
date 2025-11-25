#include <TSSP_system.h>

void TsspSystem::init() {
    for (int i = 0; i < TSSP_NUM; i++) {
        pinMode(tsspPins[i], INPUT);
    }
}

void TsspSystem::update() {
    ballStr = 0;
    uint8_t tsspValues[TSSP_NUM] = {0};

    // Sample readings
    for (int i = 0; i < 255; i++) {
        for (int j = 0; j < TSSP_NUM; j++) {
            tsspValues[j] += 1 - digitalRead(tsspPins[j]);
            delayMicroseconds(1);
        }
    }

    // Find the strongest sensor
    int maxIndex = 0;
    for (int i = 1; i < TSSP_NUM; i++) {
        if (tsspValues[i] >= tsspValues[maxIndex]) {
            maxIndex = i;
        }
    }

    // Direction and strength calculations
    ballDir = maxIndex * (360.0f / TSSP_NUM);
    ballStr = tsspValues[maxIndex];
    
}
