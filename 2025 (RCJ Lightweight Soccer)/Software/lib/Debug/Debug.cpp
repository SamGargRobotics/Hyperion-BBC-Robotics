#include <Debug.h>

void Debug::init() {
    Serial.begin(9600);
}

void Debug::update(Tssp_system* tssp, Camera* cam) {
    // Check for input
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input == "ballDir" || input == "ballStr" || input == "goalDist") {
            serialMode = input;
            Serial.println("Switched to mode: " + serialMode);
        } else if (input == "stop") {
            serialMode = "none";
            Serial.println("Stopped serial output");
        }
    }

    // Handle output
    if (serialMode == "ballDir") {
        Serial.println(tssp->getBallDir());
    } else if (serialMode == "ballStr") {
        Serial.println(tssp->getBallStr());
    } else if (serialMode == "goalDist") {
        Serial.println(cam->getDefendGoalDist());
    }
}
