#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <Configuration.h>
#include <Timer.h>

/**
 * @struct RobotData
 * @brief Represents all relevant telemetry shared between two robots through Bluetooth.
 *
 * This structure encapsulates directional and strength data for the ball, goal
 * information, robot state information, and the robot's assigned role. It is used
 * to synchronise behaviour between robots in cooperative soccer algorithms.
 */
struct RobotData {
    bool role;
    uint16_t ballDir;
    uint8_t ballStr;
    uint16_t goalAng;
    uint8_t goalDist;
    uint8_t batLvl;
    bool enabled;
};

/**
 * @class Bluetooth
 * @brief Manages all Bluetooth communication for robot-to-robot coordination.
 *
 * This class handles serial initialisation, bidirectional message exchange,
 * and determination of team roles based on received data. It uses timing
 * mechanisms to regulate connection updates and transmission frequency.
 *
 * The Bluetooth class is designed to operate in real time within a closed-loop
 * robotic soccer environment. It ensures deterministic, periodic communication
 * even under reduced performance rates defined by BT_PERFORMANCE_PERCENT.
 */
class Bluetooth {
public:
    void init();
    void update(float ballDir, float ballStr, float goalAng, float goalDist, float batLvl, bool enabled);
    bool get_role();
private:
    void read();
    void send();
    int roleWeighting(const RobotData& r);

    RobotData self = {0};
    RobotData other = {0};
    Timer connectedTimer = Timer(1000000);
    Timer sendTimer = Timer(208333/BT_PERFORMANCE_PERCENT);

    bool switching = false;
    bool otherPrevRole = false;
};

#endif