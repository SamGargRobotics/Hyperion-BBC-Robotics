#include <Kicker.h>

/**
 * @brief Initialise the kicker hardware interfaces and internal timers.
 *
 * This function configures the kicker output pin, initialises the voltage
 * divider used for capacitor monitoring, and synchronises both timing systems
 * responsible for pulse duration and recharge lockout.
 *
 * Behaviour:
 * - Sets @ref KICKER_PIN as an OUTPUT and ensures it begins in a safe LOW state.
 * - Calls @ref VoltageDivider::init to prepare ADC sampling.
 * - Calls @ref Timer::update on both @ref pulseDuration and @ref rechargeDelay
 *   to establish their reference timepoints.
 *
 * @note This must be called before any attempt to fire the kicker.
 */
void Kicker::init() {
    pinMode(KICKER_PIN, OUTPUT);
    digitalWrite(KICKER_PIN, LOW);

    kickerVoltage.init();
    pulseDuration.update();
    rechargeDelay.update();
}

/**
 * @brief Executes one update cycle of the kicker state machine.
 *
 * This function handles both **ending an existing kick pulse** and
 * **initiating a new one** if safety constraints allow.
 *
 * ### 1. Pulse termination
 * If a kick pulse is active and the configured duration has elapsed
 * (`pulseDuration.time_has_passed_no_update()`), the solenoid output is
 * switched LOW and a recharge-lockout timer is started. This prevents
 * rapid re-firing that might damage the hardware or cause power instability.
 *
 * ### 2. Kick initiation
 * A new kick will only begin if all the following conditions are met:
 * - No pulse is currently active.
 * - The capacitor voltage (sampled via @ref kickerVoltage) meets or exceeds
 *   @ref KICKER_REQUIRED_VOLT.
 * - The recharge delay since the previous kick has elapsed.
 *
 * When triggered:
 * - @ref KICKER_PIN is set HIGH.
 * - `pulseActive` becomes true.
 * - @ref pulseDuration.update() resets the timer to mark the start of the pulse.
 *
 * @note This function should be called frequently inside the main control loop.
 */
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
