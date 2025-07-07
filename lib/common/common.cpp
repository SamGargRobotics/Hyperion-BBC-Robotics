/*!
 * @file common.cpp
 */

 #include "common.h"

/*!
 * @brief Computes the floating-point modulus operation that always returns a 
 *        non-negative remainder.
 * 
 * This function calculates the remainder of `x` divided by `m`, ensuring that 
 * the result is always non-negative.
 * It correctly handles negative values of `x` by adding `m` when the remainder 
 * is negative.
 * 
 * @param x The dividend (floating-point number).
 * @param m The divisor (floating-point number).
 * @return The non-negative remainder of `x` divided by `m`.
 */

float floatMod(float x, float m) {
    float r = fmod(x, m);
    return r<0 ? r+m : r;
}

int intMod(int x, int m) {
    int r = x % m;
    return r < 0 ? r + m : r;
}
/**
 * @brief Calculates the smallest angular difference between two angles.
 *
 * This function computes the absolute difference between two angles on a circle,
 * ensuring the result is always in the range [0, 180] degrees. It accounts for the 
 * circular nature of angles, meaning that the difference between 350° and 10° is 20°, 
 * not 340°.
 *
 * @param a The first angle in degrees (typically in range [0, 360), but any float is accepted).
 * @param b The second angle in degrees (typically in range [0, 360), but any float is accepted).
 * @return The smallest difference between the two angles, in degrees.
 */
float circularDiff(float a, float b) {
    float diff = fabs(a - b);
    return (diff <= 180) ? diff : 360 - diff;
}

float angleBetween(float angleCounterClockwise, float angleClockwise) {
    return floatMod(angleClockwise - angleCounterClockwise, 360);
}

float smallestAngleBetween(float angleCounterClockwise, float angleClockwise) {
    float ang = angleBetween(angleCounterClockwise, angleClockwise);
    return fmin(ang, 360 - ang);
}

float midAngleBetween(float angleCounterClockwise, float angleClockwise) {
    return floatMod(angleCounterClockwise + angleBetween(angleCounterClockwise, angleClockwise) / 2.0, 360);
}