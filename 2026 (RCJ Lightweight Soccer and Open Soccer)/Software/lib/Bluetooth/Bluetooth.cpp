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

    // strategy here and below
}

void Bluetooth::read() {
       while(BT_SERIAL.available() >= BT_PACKET_SIZE) {
        uint8_t byte1 = BT_SERIAL.read();
        uint8_t byte2 = BT_SERIAL.peek();
        if(byte1 == BT_START_BYTE && byte2 == BT_START_BYTE) {
            BT_SERIAL.read();
            otherPrevRole = other.role;
            uint8_t info = BT_SERIAL.read();
            other.enabled = info >> 4;
            other.role = info % 2;
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