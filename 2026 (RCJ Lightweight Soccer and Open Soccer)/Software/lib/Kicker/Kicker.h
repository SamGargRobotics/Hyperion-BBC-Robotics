#ifndef KICKER_H
#define KICKER_H

#include <Arduino.h>
#include <Pins.h>
#include <Configuration.h>
#include <Voltage_divider.h>
#include <Timer.h>

class Kicker {
public:
    void init();
    void fire();
private:
    VoltageDivider kickerVoltage = VoltageDivider(KICKER_VD_PIN, 
                                                  KICKER_VOLTAGE_STABALISER);

    Timer pulseDuration = Timer(35000);
    Timer rechargeDelay = Timer(500000);

    bool pulseActive = false;
};

#endif
