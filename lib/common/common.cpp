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

/**
 * @brief Calculates the average of two angles on a circle, accounting for wrap-around at 360°.
 * 
 * This function returns the midpoint between two angles `a` and `b` (in degrees),
 * taking into account circular geometry. It correctly handles cases where the 
 * shortest path between the angles crosses the 0°/360° boundary.
 *
 * For example, the average of 350° and 10° will correctly return 0°, not 180°.
 * 
 * @param a The first angle in degrees (0 ≤ a < 360).
 * @param b The second angle in degrees (0 ≤ b < 360).
 * @return float The circular average of the two angles, normalized to the range [0, 360).
 */
float circularAverage(float a, float b) {
    float diff = fabs(a - b);
    if (diff > 180) {
        if (a > b) b += 360;
        else a += 360;
    }
    float avg = (a + b) / 2;
    return fmod(avg, 360);
}

float circularDiff(float a, float b) {
    float diff = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
    return fabs(diff);
}