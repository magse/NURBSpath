#pragma once

#include <nurbspath/config.hpp>
#include <nurbspath/nurbspath.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>

namespace test_support {

using real = double;
using nurbspath::circle2;
using nurbspath::nurbs_spline2;
using nurbspath::nurbs_spline3;
using nurbspath::plane3;
using nurbspath::point2;
using nurbspath::point3;
using nurbspath::ray2;
using nurbspath::ray3;
using nurbspath::sphere3;
using nurbspath::vector2;
using nurbspath::vector3;

inline int failure_count = 0;

inline void check(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failure_count;
    }
}

inline void check_near(
    real actual,
    real expected,
    real tolerance,
    std::string_view message) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message << " (actual " << actual
                  << ", expected " << expected << ")\n";
        ++failure_count;
    }
}

inline void check_point(
    const point3<real>& actual,
    const point3<real>& expected,
    real tolerance,
    std::string_view message) {
    check(actual.approximately_equal(expected, tolerance), message);
}

inline void check_point2(
    const point2<real>& actual,
    const point2<real>& expected,
    real tolerance,
    std::string_view message) {
    check(actual.approximately_equal(expected, tolerance), message);
}

inline nurbs_spline2<real> make_line2(
    const point2<real>& first,
    const point2<real>& second,
    real s_min = 0.0,
    real s_max = 1.0) {
    return {
        {first, second},
        {1.0, 1.0},
        {s_min, s_min, s_max, s_max},
        1
    };
}

inline nurbs_spline3<real> make_line(
    const point3<real>& first,
    const point3<real>& second,
    real s_min = 0.0,
    real s_max = 1.0) {
    return {
        {first, second},
        {1.0, 1.0},
        {s_min, s_min, s_max, s_max},
        1
    };
}

inline int finish(std::string_view test_name) {
    if (failure_count != 0) {
        std::cerr << test_name << ": " << failure_count << " check(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << test_name << " passed\n";
    return EXIT_SUCCESS;
}

} // namespace test_support
