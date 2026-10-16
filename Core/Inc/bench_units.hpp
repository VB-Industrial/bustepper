#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
namespace bench_units {
constexpr double two_pi = 6.2831853071795864769;
inline double steps_per_radian(double gear, double full_steps, double microsteps) {
    return gear * full_steps * microsteps / two_pi;
}
inline bool position_fits(double radians, double steps_per_rad) {
    const double steps = radians * steps_per_rad;
    return std::isfinite(steps) && steps >= std::numeric_limits<int32_t>::min() &&
           steps <= std::numeric_limits<int32_t>::max();
}
inline bool velocity_fits(double radians_s, double steps_per_rad, double clock) {
    const double value = std::abs(radians_s) * steps_per_rad * 16777216.0 / clock;
    return std::isfinite(value) && value <= 8388096.0;
}
inline bool acceleration_fits(double radians_s2, double steps_per_rad, double clock) {
    const double value = radians_s2 * steps_per_rad * 2199023255552.0 / (clock * clock);
    return std::isfinite(value) && value >= 0.5 && value < 65535.0;
}
} // namespace bench_units
