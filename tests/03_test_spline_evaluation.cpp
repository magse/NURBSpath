#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <cmath>
#include <stdexcept>

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
    check(line.third_derivative(4.0).is_near_zero(1e-11),
          "unit-weight linear spline third derivative");
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

    const nurbs_spline3<real> unit_cubic(
        {{0.0, 0.0, 0.0},
         {1.0, 0.0, 1.0},
         {1.0, 2.0, -1.0},
         {4.0, 3.0, 2.0}},
        {1.0, 1.0, 1.0, 1.0},
        {2.0, 2.0, 2.0, 2.0, 4.0, 4.0, 4.0, 4.0},
        3);
    const vector3<real> expected_cubic_third{3.0, -9.0 / 4.0, 6.0};
    check(unit_cubic.third_derivative(2.0).approximately_equal(
              expected_cubic_third, 1e-11) &&
              unit_cubic.third_derivative(3.0).approximately_equal(
                  expected_cubic_third, 1e-11) &&
              unit_cubic.third_derivative(4.0).approximately_equal(
                  expected_cubic_third, 1e-11),
          "cubic third derivative respects native s and endpoint spans");

    const nurbs_spline3<real> two_span_cubic(
        {{0.0, 0.0, 0.0},
         {0.0, 0.0, 0.0},
         {0.0, 0.0, 0.0},
         {1.0, 0.0, 0.0},
         {1.0, 0.0, 0.0},
         {1.0, 0.0, 0.0},
         {1.0, 2.0, 3.0}},
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0},
        3);
    check(two_span_cubic.third_derivative(0.25).approximately_equal(
              {48.0, 0.0, 0.0}, 1e-11) &&
              two_span_cubic.third_derivative(0.5).approximately_equal(
                  {0.0, 96.0, 144.0}, 1e-11) &&
              two_span_cubic.third_derivative(1.0).approximately_equal(
                  {0.0, 96.0, 144.0}, 1e-11),
          "third derivative selects the right span at an internal knot");

    const nurbs_spline3<real> rational_line(
        {{0.0, 0.0, 0.0}, {1.0, 2.0, 3.0}},
        {1.0, 2.0},
        {0.0, 0.0, 1.0, 1.0},
        1);
    check(rational_line.second_derivative(0.5).approximately_equal(
              {-32.0 / 27.0, -64.0 / 27.0, -32.0 / 9.0}, 1e-11),
          "degree-one rational spline has an analytic second derivative");
    check(rational_line.third_derivative(0.5).approximately_equal(
              {64.0 / 27.0, 128.0 / 27.0, 64.0 / 9.0}, 1e-11),
          "degree-one rational spline has an analytic third derivative");

    const nurbs_spline3<real> rational_cubic(
        {{0.0, 0.0, 0.0},
         {1.0, 0.0, 0.0},
         {0.0, 1.0, 0.0},
         {0.0, 0.0, 1.0}},
        {1.0, 1.0, 1.0, 5.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0},
        3);
    check(rational_cubic.third_derivative(0.5).approximately_equal(
              {32.0, 8.0, -80.0 / 3.0}, 1e-11),
          "rational cubic third derivative uses all quotient-rule terms");

    const nurbs_spline3<real> joined_curve(
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0},
         {2.0, 1.0, 0.0}, {2.0, 2.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0},
        2);
    check(std::isfinite(joined_curve.evaluate(0.5).x),
          "multiple knot evaluates finitely");
    check(std::isfinite(joined_curve.first_derivative(0.5).length()),
          "multiple-knot derivative evaluates finitely");

    // The active domain may begin or end inside a repeated-knot run.  The
    // outer knot keeps the multiplicity at degree + 1, so these are valid
    // unclamped layouts rather than over-multiplicity error cases.
    const nurbs_spline3<real> repeated_start_boundary(
        {{-99.0, 0.0, 0.0},
         {-9.0, 0.0, 0.0},
         {0.0, 0.0, 0.0},
         {0.25, 0.0, 0.0},
         {0.75, 0.0, 0.0},
         {1.0, 0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
        {-2.0, -1.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 2.0},
        2);
    const point3<real> repeated_start =
        repeated_start_boundary.evaluate(repeated_start_boundary.s_min());
    check(std::isfinite(repeated_start.x) && std::isfinite(repeated_start.y) &&
              std::isfinite(repeated_start.z),
          "repeated active start knot evaluates finitely");
    check_point(repeated_start, {0.0, 0.0, 0.0}, 1e-12,
                "repeated active start knot uses its right-hand span");
    check_point(repeated_start_boundary.get_start(), repeated_start, 0.0,
                "repeated active start knot updates the cached start");
    check(repeated_start_boundary.first_derivative(0.0).approximately_equal(
              {1.0, 0.0, 0.0}, 1e-12) &&
              repeated_start_boundary.second_derivative(0.0).is_near_zero(
                  1e-12) &&
              repeated_start_boundary.third_derivative(0.0).is_near_zero(
                  1e-12),
          "repeated active start knot has finite right-hand derivatives");
    bool rejected_repeated_boundary_seam = false;
    try {
        static_cast<void>(nurbs_spline3<real>(
            repeated_start_boundary.get_control_points(),
            repeated_start_boundary.get_weights(),
            repeated_start_boundary.get_knots(),
            repeated_start_boundary.degree(),
            true));
    } catch (const std::invalid_argument&) {
        rejected_repeated_boundary_seam = true;
    }
    check(rejected_repeated_boundary_seam,
          "repeated active boundary cannot hide an open seam");

    const nurbs_spline3<real> repeated_end_boundary(
        {{0.0, 0.0, 0.0},
         {0.25, 0.0, 0.0},
         {0.75, 0.0, 0.0},
         {1.0, 0.0, 0.0},
         {9.0, 0.0, 0.0},
         {99.0, 0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
        {-1.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 2.0, 3.0},
        2);
    const point3<real> repeated_end =
        repeated_end_boundary.evaluate(repeated_end_boundary.s_max());
    check(std::isfinite(repeated_end.x) && std::isfinite(repeated_end.y) &&
              std::isfinite(repeated_end.z),
          "repeated active end knot evaluates finitely");
    check_point(repeated_end, {1.0, 0.0, 0.0}, 1e-12,
                "repeated active end knot uses its left-hand span");
    check_point(repeated_end_boundary.get_end(), repeated_end, 0.0,
                "repeated active end knot updates the cached end");
    check(repeated_end_boundary.first_derivative(1.0).approximately_equal(
              {1.0, 0.0, 0.0}, 1e-12) &&
              repeated_end_boundary.second_derivative(1.0).is_near_zero(
                  1e-12) &&
              repeated_end_boundary.third_derivative(1.0).is_near_zero(
                  1e-12),
          "repeated active end knot has finite left-hand derivatives");
    return finish("03_test_spline_evaluation");
}
