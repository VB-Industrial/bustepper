#include "bench_units.hpp"
#include <cassert>
#include <cmath>
#include <limits>
int main() {
    using namespace bench_units;
    const double s = steps_per_radian(50, 200, 256);
    assert(std::abs(s * two_pi - 2560000) < 1e-8);
    assert(std::abs(steps_per_radian(1, 200, 256) * 50 - s) < 1e-8);
    assert(position_fits(0, s));
    assert(position_fits(double(INT32_MAX) / s, s));
    assert(!position_fits((double(INT32_MAX) + 1) / s, s));
    assert(!position_fits((double(INT32_MIN) - 1) / s, s));
    assert(!position_fits(std::numeric_limits<double>::infinity(), s));
    assert(!position_fits(std::numeric_limits<double>::quiet_NaN(), s));
    assert(velocity_fits(0.25, s, 12000000));
    assert(velocity_fits(-0.25, s, 12000000));
    assert(!velocity_fits(1000, s, 12000000));
    assert(!velocity_fits(std::numeric_limits<double>::quiet_NaN(), s, 12000000));
    assert(acceleration_fits(0.5, s, 12000000));
    assert(!acceleration_fits(1e-10, s, 12000000));
    assert(!acceleration_fits(1e9, s, 12000000));
}
