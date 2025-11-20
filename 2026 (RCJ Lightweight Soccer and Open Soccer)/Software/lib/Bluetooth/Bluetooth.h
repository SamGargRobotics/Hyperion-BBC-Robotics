#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <Configuration.h>
#include <Timer.h>

struct RobotData {
    bool role;
    uint16_t ballDir;
    uint8_t ballStr;
    uint16_t goalAng;
    uint8_t goalDist;
    uint8_t batLvl;
    bool enabled;
};

class Bluetooth {
public:
    void init();
    void update(float ballDir, float ballStr, float goalAng, float goalDist, float batLvl, bool enabled);
    bool get_role();
private:
    void read();
    void send();

    RobotData self = {0};
    RobotData other = {0};
    Timer connectedTimer = Timer(1000000);
    Timer sendTimer = Timer(208333/BT_PERFORMANCE_PERCENT);

    bool switching = false;
    bool otherPrevRole = false;
};

#endif