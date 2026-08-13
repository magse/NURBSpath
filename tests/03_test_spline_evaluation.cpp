#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <cmath>

int main() {
    using namespace test_support;

    const auto line = make_line(
        {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, 0.0, 10.0);
    check_point(line.evaluate(4.0), {4.0, 0.0, 0.0}, 1e-11,
                "linear spline evaluation");
    check(line.first_derivative(4.0).approximately_equal(
              {1.0, 0.0, 0.0}, 1e-11),
          "linear spline first derivative");
    check(line.second_derivative(4.0).is_near_zero(1e-11),
          "linear spline second derivative");
    check(line.tangent(4.0).approximately_equal({1.0, 0.0, 0.0}, 1e-11),
          "linear spline tangent");
    check_near(line.approximate_arc_length(), 10.0, 1e-9,
               "linear spline arc length");

    const real root_half = std::sqrt(0.5);
    const nurbs_spline3<real> quarter_circle(
        {{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        2);
    check_point(quarter_circle.evaluate(0.5), {root_half, root_half, 0.0}, 1e-11,
                "rational quarter circle evaluation");

    const nurbs_spline3<real> joined_curve(
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0},
         {2.0, 1.0, 0.0}, {2.0, 2.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0},
        2);
    check(std::isfinite(joined_curve.evaluate(0.5).x()),
          "repeated knot evaluates finitely");
    check(std::isfinite(joined_curve.first_derivative(0.5).length()),
          "repeated-knot derivative evaluates finitely");
    return finish("03_test_spline_evaluation");
}
