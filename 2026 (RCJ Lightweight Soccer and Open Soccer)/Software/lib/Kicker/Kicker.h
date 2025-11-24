#ifndef KICKER_H
#define KICKER_H

#include <Arduino.h>
#include <Pins.h>
#include <Configuration.h>
#include <Voltage_divider.h>
#include <Timer.h>

/**
 * @class Kicker
 * @brief Controls the robot's electromagnetic kicking mechanism.
 *
 * The Kicker module manages charging, firing, and enforcing timing rules
 * for the solenoid-based kicking system. It tracks capacitor voltage via a
 * voltage divider and uses high-precision microsecond timers to ensure that
 * each kick pulse has a safe and consistent duration.
 *
 * ## Functional Overview
 * - **Voltage Monitoring**  
 *   Continuously samples the kicker capacitor through a @ref VoltageDivider
 *   to determine whether sufficient voltage is available for firing.
 *
 * - **Pulse Control**  
 *   Uses @ref Timer pulseDuration to apply a controlled high-power pulse to
 *   the kicker output circuit. The pulse length (in microseconds) is tuned
 *   to ensure effective ball contact without damaging hardware.
 *
 * - **Recharge Lockout**  
 *   After each kick, @ref Timer rechargeDelay prevents re-firing until the
 *   capacitor has had enough time to safely recharge.
 */
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
