/*!
 * @file bluetooth.cpp
 */
#include <Bluetooth.h>

/*!
 * @brief Initializes the module for use accross the library and code
 */
void Bluetooth::init() {
    BT_SERIAL.begin(BT_BAUD);
    connectedTimer.resetTime();
    roleConflict.resetTime();
    sendTimer.resetTime();
}

/*!
 * @brief Main controlling function, determines when the HC-05 interacts with 
 *        other modules.
 * @param logic Attack or defend state
 * @param ballDir Current Direction of Ball.
 * @param ballDis Current Distance of Ball away from robot (cm).
 */
void Bluetooth::update(float ballDir, float ballStr) {
    self.ballDir = ballDir;
    self.ballStr = ballStr;
    if(sendTimer.timeHasPassed()) {
        send();
    }
    read();

    bool connected = !connectedTimer.timeHasPassedNoUpdate();
    
    if(!connected) {
        self.role = 0;
        roleConflict.resetTime();
    } else if(switching) {
        self.role = !self.role;
        roleConflict.resetTime();
    } else if(self.role == other.role) {
        if(roleConflict.timeHasPassedNoUpdate()) {
            self.role = self.ballStr > other.ballStr;
            roleConflict.resetTime();
        }
    } else if(!self.role && (self.ballStr > SURGE_STR_VALUE) && (self.ballDir < 15 || self.ballDir > 345)) {
        switching = true;
    }
}

/*!
 * @brief If the serial has more than one full packet, it attempts to read the
          bluetooth module.
 * 
 */
void Bluetooth::read() {
   if(BT_SERIAL.available() >= BT_PACKET_SIZE) {
        uint8_t byte1 = BT_SERIAL.read();
        uint8_t byte2 = BT_SERIAL.peek();
        if(byte1 == BT_START_BYTE && byte2 == BT_START_BYTE) {
            BT_SERIAL.read();
            bool otherPrevRole = other.role;
            other.role = BT_SERIAL.read();
            switching = (otherPrevRole != other.role) && (self.role == other.role);

            byte1 = BT_SERIAL.read();
            byte2 = BT_SERIAL.read();
            uint16_t tempBallDir = (byte1 << 8) | byte2;
            other.ballDir = tempBallDir / 100.0f;
            other.ballStr = BT_SERIAL.read();
            connectedTimer.resetTime();
        }
    }
}

/*!
 * @brief Writes data to the bluetooth module for other bluetooth devices to
 *        read off (other HC-05's)
 * @param batLevel Current battery level of the robot.
 * @param ballDir Current direction of the ball.
 * @param ballDis Current distance of the ball away from the robot (cm).
 */
void Bluetooth::send() {
    BT_SERIAL.write(BT_START_BYTE);
    BT_SERIAL.write(BT_START_BYTE);
    BT_SERIAL.write(self.role);
    uint16_t b = self.ballDir * 100;
    BT_SERIAL.write(highByte(b));
    BT_SERIAL.write(lowByte(b));
    BT_SERIAL.write(self.ballStr);
}

bool Bluetooth::getRole() {
    return self.role;
}