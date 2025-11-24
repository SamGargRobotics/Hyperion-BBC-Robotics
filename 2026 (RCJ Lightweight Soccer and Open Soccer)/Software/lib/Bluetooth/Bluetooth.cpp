#include <Bluetooth.h>

/**
 * @brief Initialises the Bluetooth interface and resets internal timers.
 *
 * This sets up the serial communication at the configured baud rate
 * (defined by @ref BT_BAUD) and primes both the connection timer and
 * transmission timer. This must be called once during system startup.
 *
 * @return void
 */
void Bluetooth::init() {
    BT_SERIAL.begin(BT_BAUD);
    connectedTimer.update();
    sendTimer.update();
}

/**
 * @brief Updates local telemetry, handles sending, reading, and role resolution.
 *
 * This function is called every loop cycle. It performs:
 *  - Updating outbound robot information (ball, goal, battery, enabled state)
 *  - Periodic transmission via @ref sendTimer
 *  - Incoming packet parsing via @ref read
 *  - Role determination logic based on: ball visibility, teammate connectivity,
 *    switching state, battery fallback, and calculated weighting via @ref 
 *    roleWeighting.
 *
 * The resulting role is stored in @ref self.role.
 *
 * @param ballDir  Ball direction in degrees (0–359).
 * @param ballStr  Ball signal strength (0–255).
 * @param goalAng  Goal angle in degrees (0–359).
 * @param goalDist Estimated distance to goal (0–255).
 * @param batLvl   Battery level percentage (0–100).
 * @param enabled  Robot enabled state.
 *
 * @return void
 */
void Bluetooth::update(float ballDir, float ballStr, float goalAng, float goalDist, float batLvl, bool enabled) {
    self.ballDir = ballDir;
    self.ballStr = ballStr;
    self.goalAng = goalAng;
    self.goalDist = goalDist;
    self.batLvl = batLvl;
    self.enabled = enabled;
    if(sendTimer.time_has_passed()) {
        send();
    }
    read();

    bool connected = !connectedTimer.time_has_passed_no_update();

    // If either robot doesn’t see the ball at all → stop switching & keep roles stable.
    if (self.ballStr == 0 && other.ballStr == 0) {
        switching = false;
        return;
    } if (!self.enabled) {
        self.role = 1;
        return;
    } if (!connected || !other.enabled) {
        self.role = 0;
        return;
    } if (switching) {
        self.role = !self.role;
        switching = false;
        return;
    } if (self.batLvl < 11 && other.batLvl < 11 && self.batLvl != other.batLvl) {
        self.role = (self.batLvl > other.batLvl);  
        return;
    }

    int sSelf  = roleWeighting(self);
    int sOther = roleWeighting(other);
    self.role = (sSelf >= sOther);
}

/**
 * @brief Computes the "importance rating" of a robot for striker role selection.
 *
 * This method uses a weighted scoring system based on:
 *  - Ball strength (0–40 pts)
 *  - Ball attack cone alignment (+20 pts)
 *  - Goal alignment (+15 pts)
 *  - Distance to goal (0–15 pts)
 *  - Battery level (+0–10 pts)
 *
 * The robot with the higher score becomes striker.
 *
 * @param r Reference to the robot's telemetry state.
 * @return int Calculated weighting score.
 */
int Bluetooth::roleWeighting(const RobotData& r) {
    // -------- BALL STRENGTH (0–40 pts) --------
    int score = r.ballStr * 0.4f;
    // -------- ATTACK CONE (0 or 20 pts) -------
    score += (r.ballDir <= 30 || r.ballDir >= 330) ? 20 : 0;
    // -------- GOAL ALIGNMENT (0 or 15 pts) ---- 
    score += (r.goalAng <= 25 || r.goalAng >= 335) ? 15 : 0;
    // -------- DISTANCE FROM OWN GOAL (0–15) ----
    score += r.goalDist * 0.15f;
    // -------- BATTERY LEVEL (0–10 pts) ---------
    score += r.batLvl * 1.0f;
    return score;
}

/**
 * @brief Reads and parses Bluetooth packets from the teammate robot.
 *
 * Valid packets begin with two @ref BT_START_BYTE markers.  
 * Once detected, the function extracts the teammate's: enabled state, role,
 * ball direction, ball strength, goal angle, goal distance, battery level.
 *
 * When a valid packet is received, @ref connectedTimer is reset,
 * meaning the robots are still in communication.
 *
 * Additionally, if the teammate’s role changes and matches our own,
 * the system enters "switching" mode, triggering a coordinated role swap.
 *
 * @return void
 */
void Bluetooth::read() {
       while(BT_SERIAL.available() >= BT_PACKET_SIZE) {
        uint8_t byte1 = BT_SERIAL.read();
        uint8_t byte2 = BT_SERIAL.peek();
        if(byte1 == BT_START_BYTE && byte2 == BT_START_BYTE) {
            BT_SERIAL.read();
            otherPrevRole = other.role;
            uint8_t info = BT_SERIAL.read();
            other.enabled = (info >> 4) & 0x01;
            other.role = info & 0x01;
            switching = (otherPrevRole != other.role) && (self.role == other.role);
            other.ballDir = BT_SERIAL.read();
            other.ballStr = BT_SERIAL.read();
            other.goalAng = BT_SERIAL.read();
            other.goalDist = BT_SERIAL.read();
            other.batLvl = BT_SERIAL.read();
            connectedTimer.update();
        }
    }
}

/**
 * @brief Sends this robot's telemetry data through Bluetooth.
 *
 * The packet format is:
 *  [START][START][INFO][ballDir][ballStr][goalAng][goalDist][batLvl]
 *
 * Where INFO encodes:
 *  - bit 4 → enabled flag  
 *  - bit 0 → role flag  
 *
 * Transmission frequency is controlled by @ref sendTimer.
 *
 * @return void
 */
void Bluetooth::send() {
    BT_SERIAL.write(BT_START_BYTE);
    BT_SERIAL.write(BT_START_BYTE);
    uint8_t info = (self.enabled&0x01) << 4 | (self.role&0x01);
    BT_SERIAL.write(info);
    BT_SERIAL.write(self.ballDir);
    BT_SERIAL.write(self.ballStr);
    BT_SERIAL.write(self.goalAng);
    BT_SERIAL.write(self.goalDist);
    BT_SERIAL.write(self.batLvl);
}

/**
 * @brief Returns this robot's currently assigned role.
 *
 * @return true  Robot is striker.
 * @return false Robot is defender.
 */
bool Bluetooth::get_role() {
    return self.role;
}