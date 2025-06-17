/*! 
 * @file Timer.cpp
 */
#include "Timer.h"

/*! 
 * @brief Initialize the timer based on the users constraints.
 */
Timer::Timer(unsigned long duration) {
    timerDuration = duration;
}

/*! 
 * @brief Function to reset the timer based on the current micros.
 */
void Timer::resetTime() {
    lastUpdate = micros();
}

/*! 
 * @brief Check if the set time has passed since the last reset time.
 */
bool Timer::timeHasPassed() {
    if (micros() - lastUpdate > timerDuration) {
        resetTime();
        return true;
    }

    return false;
}

/*!
 * @brief Check if the set time has passed since the last reset time, but do not
 *        reset the timer.
 */
bool Timer::timeHasPassedNoUpdate() {
    return micros() - lastUpdate > timerDuration;
}