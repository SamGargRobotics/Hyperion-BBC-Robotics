#include <Kicker.h>

void Kicker::init() {
    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, LOW);

    kickerVoltage.init();
    pulseDuration.update();
    rechargeDelay.update();
}

void Kicker::fire() {
    kickerVoltage.update();

    // ------ END KICK WHEN TIME IS UP ------
    if (pulseActive && pulseDuration.time_has_passed_no_update()) {
        pulseActive = false;
        digitalWrite(KICKER_PIN, LOW);
        rechargeDelay.update();   // cooldown starts after pulse ends
        return;
    }

    // ------ KICK CONDITIONS ------
    if (!pulseActive && (kickerVoltage.get_lvl() >= KICKER_REQUIRED_VOLT) && 
        rechargeDelay.time_has_passed_no_update()) {
        digitalWrite(KICKER_PIN, HIGH);
        pulseActive = true;
        pulseDuration.update();  // start pulse timer
    }
}
