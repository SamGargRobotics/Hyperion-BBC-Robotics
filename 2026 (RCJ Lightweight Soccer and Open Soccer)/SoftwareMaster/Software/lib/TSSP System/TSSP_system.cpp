#include <Arduino.h>
#include <TSSP_system.h>

void TsspSystem::init() {
    Serial1.begin(115200);   // RX from slave teensy
}

void TsspSystem::update() {
    readUartData();
    float x = cos(bInfo._dir);
    float y = sin(bInfo._dir);
    bInfo._mag = sqrt(pow(x, 2) + pow(y, 2));
    mInfo._dir = fmod(bInfo._dir + (((bInfo._dir > 180 ? bInfo._dir - 360 : bInfo._dir) < 0) ? 
              -(constrain(0.02f * constrain(bInfo._str / ORBIT_TUNER, 0, 1) * 
              expf(4.5f * constrain(bInfo._str / ORBIT_TUNER, 0, 1)), 0.0f, 1.0f) *
              min(0.4f * expf(0.25f * abs(bInfo._dir > 180 ? bInfo._dir - 360 : 
              bInfo._dir)) - 0.4f, 90.0f)) : (constrain(0.02f * constrain(bInfo._str /
              ORBIT_TUNER, 0, 1) * expf(4.5f * constrain(bInfo._str / ORBIT_TUNER, 
              0, 1)), 0.0f, 1.0f) * min(0.4f * expf(0.25f * abs(bInfo._dir > 180 ? 
              bInfo._dir - 360 : bInfo._dir)) - 0.4f, 90.0f))), 360.0f);
    mInfo._spd = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0f - ((constrain(
              0.02f * constrain(bInfo._str / ORBIT_TUNER, 0, 1) * expf(4.5f * 
              constrain(bInfo._str / ORBIT_TUNER, 0, 1)), 0.0f, 1.0f) * min(0.4f * 
              expf(0.25f * abs(bInfo._dir > 180 ? bInfo._dir - 360 : bInfo._dir)) - 0.4f,
              90.0f)) / 90.0f));
}

void TsspSystem::readUartData() {
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n') {
            // Full line received — parse
            int dirIndex = uartBuffer.indexOf("DIR:");
            int commaIndex = uartBuffer.indexOf(",");
            int strIndex = uartBuffer.indexOf("STR:");

            if (dirIndex != -1 && commaIndex != -1 && strIndex != -1) {
                bInfo._dir = uartBuffer.substring(dirIndex + 4, commaIndex).toFloat();
                bInfo._str = uartBuffer.substring(strIndex + 4).toFloat();
            }
            uartBuffer = ""; // clear for next packet
        }
        else {
            uartBuffer += c;
        }
    }
}
