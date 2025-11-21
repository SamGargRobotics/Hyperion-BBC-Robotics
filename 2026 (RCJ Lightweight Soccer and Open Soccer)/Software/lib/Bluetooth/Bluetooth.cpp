#include <Bluetooth.h>

/*!
 * @brief Initializes bluetooth module for usage.
 */
void Bluetooth::init() {
    BT_SERIAL.begin(BT_BAUD);
    connectedTimer.update();
    sendTimer.update();
}

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

/*! 
 * @brief Get function for role of robot (Attack/Defend).
 * 
 * @returns Role of the robot.
 */
bool Bluetooth::get_role() {
    return self.role;
}