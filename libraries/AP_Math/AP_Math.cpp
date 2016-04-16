#include "AP_Math.h"
#include <float.h>

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

/*
  linear interpolation based on a variable in a range
 */
float linear_interpolate(float low_output, float high_output,
                         float var_value,
                         float var_low, float var_high)
{
    if (var_value <= var_low) {
        return low_output;
    }
    if (var_value >= var_high) {
        return high_output;
    }
    float p = (var_value - var_low) / (var_high - var_low);
    return low_output + p * (high_output - low_output);
}

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_360_cd(const T &angle) -> decltype(wrap_360(angle, 100.f)) {
    return wrap_360(angle, 100.f);
}

template auto  wrap_360_cd<float>   (const float        &angle) -> decltype(wrap_360(angle, 100.f));
template auto  wrap_360_cd<double>  (const double       &angle) -> decltype(wrap_360(angle, 100.f));
template auto  wrap_360_cd<int16_t> (const int16_t      &angle) -> decltype(wrap_360(angle, 100.f));
template auto  wrap_360_cd<int32_t> (const int32_t      &angle) -> decltype(wrap_360(angle, 100.f));

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_180_cd(const T &angle) -> decltype(wrap_180(angle, 100.f)) {
    return wrap_180(angle, 100.f);
} 

template auto  wrap_180_cd<float>   (const float        &angle) -> decltype(wrap_180(angle, 100.f));
template auto  wrap_180_cd<double>  (const double       &angle) -> decltype(wrap_180(angle, 100.f));
template auto  wrap_180_cd<int16_t> (const int16_t      &angle) -> decltype(wrap_180(angle, 100.f));
template auto  wrap_180_cd<int32_t> (const int32_t      &angle) -> decltype(wrap_180(angle, 100.f));
