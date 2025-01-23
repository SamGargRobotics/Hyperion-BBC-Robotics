#ifndef TSSP_SYSTEM_H
#define TSSP_SYSTEM_H

#include <Arduino.h>
#include <configandpins.h>

class Tssp_system {
public:
    Tssp_system() {};
    void init();
    void read();
    int deviationReading(int midTssp);
    int ballStr = 0;
    int ballDir = 0;
    int highestTssp = 0;
private:
    bool deviationFunctionToggle = false;
    bool leadingTsspBigger = false;
    int readTssp[TSSPNUM] = {0};
    int tsspPins[TSSPNUM] = {TSSP1, TSSP2, TSSP3, TSSP4, TSSP5, TSSP6, TSSP7, TSSP8, TSSP9, TSSP10, TSSP11, TSSP12, TSSP13, TSSP14, TSSP15, TSSP16};
    int readingTsspIgnores[TSSPNUM] = {0};
    int largestReading = 0;
    int addedAngles = 0;
    int midTsspReading = 0;
    int followingTsspReading = 0;
    int leadingTsspReading = 0;
    int deviationOffset = 0;
    int ballDirOffset = 0;
};

#endif