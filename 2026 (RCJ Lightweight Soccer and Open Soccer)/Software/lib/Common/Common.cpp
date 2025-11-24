#include <Common.h>

/**
 * @brief Calculates the directional angle from one angle to another in a clockwise direction.
 *
 * Returns the angular distance from `angleCounterClockwise` to `angleClockwise`, measured clockwise,
 * wrapped within the range [0, 360).
 *
 * @param angleCounterClockwise The starting angle (in degrees).
 * @param angleClockwise The target angle (in degrees).
 * 
 * @returns The clockwise angle difference between the two angles (in degrees).
 */
float angleBetween(float angleCounterClockwise, float angleClockwise) {
    return fmod(angleClockwise - angleCounterClockwise, 360);
}

/**
 * @brief Calculates the midpoint angle between two angles.
 *
 * Returns the angle exactly halfway between `angleCounterClockwise` and `angleClockwise`,
 * moving in a clockwise direction from `angleCounterClockwise`.
 *
 * @param angleCounterClockwise The starting angle (in degrees).
 * @param angleClockwise The ending angle (in degrees).
 * 
 * @returns The midpoint angle between the two given angles (in degrees), wrapped to [0, 360).
 */
float Common::midAngleBetween(float angleCounterClockwise, float angleClockwise) {
    return fmod(angleCounterClockwise + angleBetween(angleCounterClockwise, angleClockwise) / 2.0, 360);
}

/**
 * @brief Computes a positive modulo result for integers.
 *
 * Returns the result of `x mod m`, ensuring the result is always in the range [0, m).
 * This handles negative `x` values correctly, unlike the standard `%` operator.
 *
 * @param x The dividend (can be negative).
 * @param m The modulus (must be positive).
 * 
 * @returns The positive remainder of `x` modulo `m`.
 */
int Common::intMod(int x, int m) {
    int r = x % m;
    return r < 0 ? r + m : r;
}