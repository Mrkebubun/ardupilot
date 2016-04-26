/*
 * This file is the current standard math implementation until the "DBL_MATH" build flag is set.
 * With this template implementation all double promotions are avoided, 
 * which can happen because of lazy integer usage in ArduPilot.
 */

#include "AP_Math_Float.h"


/* 
 * @brief: Checks whether two floats are equal
 */
template <class FloatOne, class FloatTwo>
bool is_equal(const FloatOne fVal1, const FloatTwo fVal2) {
    static_assert(std::is_arithmetic<FloatOne>::value, "ERROR - is_equal(): template parameter not of type float or int\n");
    static_assert(std::is_arithmetic<FloatTwo>::value, "ERROR - is_equal(): template parameter not of type float or int\n");
    return fabsf(fVal1 - fVal2) < std::numeric_limits<decltype(fVal1 - fVal2)>::epsilon() ? true : false; 
}

template bool is_equal<int>(const int fVal1, const int fVal2);
template bool is_equal<short>(const short fVal1, const short fVal2);
template bool is_equal<float>(const float fVal1, const float fVal2);

/* 
 * @brief: Checks whether a float is zero
 */
template <class T>
bool is_zero(const T fVal) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - is_zero(): template parameter not of type float or int\n");
    return fabsf(fVal) < std::numeric_limits<T>::epsilon() ? true : false; 
}

template bool is_zero<int>(const int fVal);
template bool is_zero<short>(const short fVal);
template bool is_zero<float>(const float fVal);

/*
 * A varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is returned.
 */
template <class T>
float safe_asin(const T &v) {
    if (isnan(static_cast<float>(v))) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return static_cast<float>(M_PI_2);
    }
    if (v <= -1.0f) {
        return static_cast<float>(-M_PI_2);
    }
    return asinf(static_cast<float>(v));
}

template float safe_asin<int>(const int &v);
template float safe_asin<short>(const short &v);
template float safe_asin<float>(const float &v);

/* 
 * A varient of sqrt() that checks the input ranges and ensures a 
 * valid value as output. If a negative number is given then 0 is returned. 
 * The reasoning is that a negative number for sqrt() in our 
 * code is usually caused by small numerical rounding errors, so the 
 * real input should have been zero
 */
template <class T>
float safe_sqrt(const T &v) {
    float ret = sqrt(static_cast<float>(v));
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

template float safe_sqrt<int>(const int &v);
template float safe_sqrt<short>(const short &v);
template float safe_sqrt<float>(const float &v);

/* 
 * @brief: Constrains an euler angle to be within the range: -180 to 180 degrees
 */
template <class T>
float wrap_180(const T &angle, float unit_mod = 1) {   
    const float ang_180 = 180.f*unit_mod;
    const float ang_360 = 360.f*unit_mod;
    float res = fmod(static_cast<float>(angle) + ang_180, ang_360);
    if (res < 0 || is_zero(res)) {
        res += ang_180;
    }
    else {
        res -= ang_180;
    }
    return res;
}

template float wrap_180<int>(const int &angle, float unit_mod = 1);
template float wrap_180<short>(const short &angle, float unit_mod = 1);
template float wrap_180<float>(const float &angle, float unit_mod = 1);

/* 
 * @brief: Constrains an euler angle to be within the range: 0 to 360 degrees
 * The second parameter changes the units. Standard: 1 == degrees, 10 == dezi, 100 == centi ..
 */
template <class T>
float wrap_360(const T &angle, float unit_mod = 1) {
    const float ang_360 = 360.f*unit_mod;
    float res = fmod(static_cast<float>(angle), ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return res;
}

template float wrap_360<int>(const int &angle, float unit_mod = 1);
template float wrap_360<short>(const short &angle, float unit_mod = 1);
template float wrap_360<float>(const float &angle, float unit_mod = 1);

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_360_cd(const T &angle) -> decltype(wrap_360(angle, 100.f)) {
    return wrap_360(angle, 100.f);
}

template auto  wrap_360_cd<float>   (const float  &angle) -> decltype(wrap_360(angle, 100.f));
template auto  wrap_360_cd<int>     (const int    &angle) -> decltype(wrap_360(angle, 100.f));
template auto  wrap_360_cd<short>   (const short  &angle) -> decltype(wrap_360(angle, 100.f));

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_180_cd(const T &angle) -> decltype(wrap_180(angle, 100.f)) {
    return wrap_180(angle, 100.f);
}

template auto  wrap_180_cd<float>   (const float  &angle) -> decltype(wrap_180(angle, 100.f));
template auto  wrap_180_cd<int>     (const int    &angle) -> decltype(wrap_180(angle, 100.f));
template auto  wrap_180_cd<short>   (const short  &angle) -> decltype(wrap_180(angle, 100.f));

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <class T>
float wrap_PI(const T &radian) {
    float res = fmod(static_cast<float>(radian) + M_PI, M_2PI);
    if (res < 0 || is_zero(res)) {
        res += M_PI;
    }
    else {
        res -= M_PI;
    }
    return res;
}

template float wrap_PI<int>(const int &radian);
template float wrap_PI<short>(const short &radian);
template float wrap_PI<float>(const float &radian);

/*
 * wrap an angle in radians to 0..2PI
 */
template <class T>
float wrap_2PI(const T &radian) {
    float res = fmod(static_cast<float>(radian), M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

template float wrap_2PI<int>(const int &radian);
template float wrap_2PI<short>(const short &radian);
template float wrap_2PI<float>(const float &radian);

/*
 * @brief: Constrains a value to be within the range: low and high
 */
template <class T>
T constrain_value(const T &amt, const T &low, const T &high) {
    if (isnan(low) || isnan(high)) {
        return amt;
    }
    return amt < low ? low : (amt > high ? high : amt);
}

float constrain_float(float const & v,float const & low, float const & high) { return constrain_value(v,low,high);}
int16_t constrain_int16(int16_t v,int16_t low, int16_t high) { return constrain_value(v,low,high);}
int32_t constrain_int32(int32_t v,int32_t low, int32_t high) { return constrain_value(v,low,high);}

//template int constrain_value<int>(const int &amt, const int &low, const int &high);
//template short constrain_value<short>(const short &amt, const short &low, const short &high);
//template float constrain_value<float>(const float &amt, const float &low, const float &high);

