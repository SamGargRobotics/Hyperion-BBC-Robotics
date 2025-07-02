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

/*!
 * @brief Calculates the minimal angular difference between two angles in degrees.
 * 
 * This function computes the smallest difference between two angles, `a` and `b`,
 * measured in degrees. The result is always in the range [0, 180], representing
 * the shortest rotation between the two angles, accounting for circular wrap-around.
 * 
 * For example, the difference between 350° and 10° is 20°, not 340°.
 * 
 * @param a The first angle in degrees.
 * @param b The second angle in degrees.
 * @return The minimal difference between the two angles, in degrees.
 */
float angleDiff(float a, float b) {
    float diff = floatMod(fabs(a - b), 360);
    return diff > 180 ? 360 - diff : diff;
}