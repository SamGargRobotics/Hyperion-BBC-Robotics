/*!
 * @file bluetooth.cpp
 */
#include <bluetooth.h>

/*!
 * @brief Initializes the module for use accross the library and code
 */
void Bluetooth::init() {
    BLUETOOTH_SERIAL.begin(BLUETOOTH_BAUD);
    last_received_time = micros();
    last_sent_time = micros();
}

/*!
 * @brief Main controlling function, determines when the HC-05 interacts with 
 *        other modules.
 * @param logic Attack or defend state
 * @param ballDir Current Direction of Ball.
 * @param ballDis Current Distance of Ball away from robot (cm).
 */
void Bluetooth::update(bool logic, float ballDir, float ballDis) {
    unsigned long current_time = micros();
    if(current_time - last_sent_time > 250000) {
        send(logic, ballDir, ballDis);
    }
    read_data = read();
    if(read_data) {
        last_received_time = micros();
    }
    prevBallStr = otherRobotBallLocation[1];
    current_time = micros();
    connection = current_time - last_received_time > 2000000 ? false : true;
}

/*!
 * @brief If the serial has more than one full packet, it attempts to read the
          bluetooth module.
 * 
 * @return If there is any data in the packet.
 */
bool Bluetooth::read() {
    // See if serial has more than one full packet
    while(BLUETOOTH_SERIAL.available() >= BLUETOOTH_PACKET_SIZE) {
        // Checks if the serial has the associatde starting BYTE
        if(BLUETOOTH_SERIAL.read() == BLUETOOTH_START_BYTE) {
            for(int i = 0; i < BLUETOOTH_PACKET_SIZE - 1; i++) {
                // Reads information sent on the serial
                bluetoothBuffer[i] = BLUETOOTH_SERIAL.read();
            }
            // Assigned associated public variables with information read from 
            // the serial
            otherRobotBallLocation[0] = (bluetoothBuffer[0] == \
                                BLUETOOTH_NO_DATA ? -1 : bluetoothBuffer[0]);
            otherRobotBallLocation[1] = (bluetoothBuffer[0] == \
                                BLUETOOTH_NO_DATA ? -1 : bluetoothBuffer[1]);
            otherRobotLogic = (bluetoothBuffer[2] == BLUETOOTH_NO_DATA \
                                ? -1 : bluetoothBuffer[2]);
            // Returns true if the data from the serial was read.
            return true;
        }
    }
    // Returns false if the data was not read at all.
    return false;
}

/*!
 * @brief Writes data to the bluetooth module for other bluetooth devices to
 *        read off (other HC-05's)
 * @param batLevel Current battery level of the robot.
 * @param ballDir Current direction of the ball.
 * @param ballDis Current distance of the ball away from the robot (cm).
 */
void Bluetooth::send(bool logic, float ballDir, float ballDis) {
    BLUETOOTH_SERIAL.write(BLUETOOTH_START_BYTE);
    BLUETOOTH_SERIAL.write(int(ballDir));
    BLUETOOTH_SERIAL.write(int(ballDis));
    BLUETOOTH_SERIAL.write(logic);
    last_sent_time = micros();
}