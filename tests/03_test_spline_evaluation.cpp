#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

int main() {
    using namespace test_support;

    const auto throws_invalid_argument = [](const auto& operation) {
        try {
            operation();
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    };
    const auto throws_overflow_error = [](const auto& operation) {
        try {
            operation();
        } catch (const std::overflow_error&) {
            return true;
        }
        return false;
    };
    const auto throws_domain_error = [](const auto& operation) {
        try {
            operation();
        } catch (const std::domain_error&) {
            return true;
        }
        return false;
    };

    const point3<real> standard_start{1.0, -2.0, 3.0};
    const point3<real> standard_end{4.0, 2.0, 3.0};
    constexpr std::size_t standard_control_count = 6;
    constexpr std::size_t standard_degree = 3;
    constexpr real standard_tolerance = 4e-10;
    const real standard_distance = distance(standard_start, standard_end);
    const nurbs_spline3<real> standard_spline(
        standard_start,
        standard_end,
        standard_control_count,
        standard_degree,
        false,
        standard_tolerance);
    const nurbs_spline3<real> defaulted_standard_spline(
        standard_start,
        standard_end,
        standard_control_count,
        standard_degree);

    check(!defaulted_standard_spline.is_closed() &&
              defaulted_standard_spline.tolerance() ==
                  real(64) * std::numeric_limits<real>::epsilon(),
          "standard 3D constructor defaults closure and tolerance");

    check(standard_spline.get_control_points().size() ==
                  standard_control_count &&
              standard_spline.get_weights().size() == standard_control_count &&
              standard_spline.degree() == standard_degree &&
              !standard_spline.is_closed() &&
              standard_spline.tolerance() == standard_tolerance,
          "standard 3D constructor preserves counts and open metadata");
    for (std::size_t index = 0; index < standard_control_count; ++index) {
        const real fraction = static_cast<real>(index) /
            static_cast<real>(standard_control_count - 1);
        check_point(
            standard_spline.get_control_point(index),
            lerp(standard_start, standard_end, fraction),
            0.0,
            "standard 3D constructor uses exact point lerp controls");
        check(standard_spline.get_weight(index) == 1.0,
              "standard 3D constructor uses unit weights");
    }
    const std::vector<real> standard_knots{
        0.0,
        0.0,
        0.0,
        0.0,
        standard_distance / 3.0,
        2.0 * standard_distance / 3.0,
        standard_distance,
        standard_distance,
        standard_distance,
        standard_distance};
    check(standard_spline.get_knots().size() == standard_knots.size(),
          "standard 3D constructor creates the required knot count");
    for (std::size_t index = 0; index < standard_knots.size(); ++index) {
        check_near(
            standard_spline.get_knot(index),
            standard_knots[index],
            1e-14,
            "standard 3D constructor creates standard distance knots");
    }
    check_near(standard_spline.s_min(), 0.0, 0.0,
               "standard 3D constructor starts its domain at zero");
    check_near(standard_spline.s_max(), standard_distance, 0.0,
               "standard 3D constructor ends its domain at point distance");
    check_point(standard_spline.get_start(), standard_start, 0.0,
                "standard 3D constructor caches its first endpoint");
    check_point(standard_spline.get_end(), standard_end, 1e-14,
                "standard 3D constructor caches its second endpoint");
    check_point(
        standard_spline.get_start(),
        standard_spline.evaluate(standard_spline.s_min()),
        0.0,
        "standard 3D constructor start cache matches evaluation");
    check_point(
        standard_spline.get_end(),
        standard_spline.evaluate(standard_spline.s_max()),
        0.0,
        "standard 3D constructor end cache matches evaluation");

    constexpr float converted_standard_tolerance = 3e-6F;
    const nurbs_spline3<real> standard_tolerance_overload(
        standard_start,
        standard_end,
        standard_control_count,
        standard_degree,
        converted_standard_tolerance);
    check(!standard_tolerance_overload.is_closed() &&
              standard_tolerance_overload.tolerance() ==
                  static_cast<real>(converted_standard_tolerance),
          "standard 3D numeric argument selects tolerance, not closure");

    check(throws_invalid_argument([&] {
              static_cast<void>(
                  nurbs_spline3<real>(standard_start, standard_end, 3, 3));
          }),
          "standard 3D constructor rejects a count not above degree");
    check(throws_invalid_argument([&] {
              static_cast<void>(
                  nurbs_spline3<real>(standard_start, standard_end, 3, 0));
          }),
          "standard 3D constructor rejects degree zero");
    check(throws_invalid_argument([&] {
              static_cast<void>(
                  nurbs_spline3<real>(standard_start, standard_start, 4, 3));
          }),
          "standard 3D constructor rejects a zero-distance knot domain");
    check(throws_overflow_error([&] {
              static_cast<void>(nurbs_spline3<real>(
                  standard_start,
                  standard_end,
                  std::numeric_limits<std::size_t>::max(),
                  1));
          }),
          "standard 3D constructor rejects knot-count overflow before allocation");
    check(throws_invalid_argument([&] {
              static_cast<void>(nurbs_spline3<real>(
                  standard_start,
                  standard_end,
                  standard_control_count,
                  standard_degree,
                  true));
          }),
          "standard 3D constructor rejects a distinct closed seam");

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
    check_near(line.curvature(4.0), 0.0, 0.0,
               "linear spline has zero curvature");
    check(throws_invalid_argument([&] {
              static_cast<void>(line.curvature(4.0, -1.0));
          }) &&
              throws_invalid_argument([&] {
                  static_cast<void>(line.curvature(
                      4.0, std::numeric_limits<real>::quiet_NaN()));
              }),
          "3D curvature rejects invalid tangent tolerances");
    check(throws_domain_error([&] {
              static_cast<void>(line.curvature(4.0, 1.0));
          }),
          "3D curvature rejects speed equal to its tangent tolerance");
    check_near(line.approximate_arc_length(), 10.0, 1e-9,
               "linear spline arc length");

    const nurbs_spline3<real> stationary(
        {{2.0, 3.0, 4.0}, {2.0, 3.0, 4.0}},
        {1.0, 1.0},
        {0.0, 0.0, 1.0, 1.0},
        1);
    check(throws_domain_error([&] {
              static_cast<void>(stationary.curvature(0.5));
          }),
          "3D curvature rejects a stationary spline point");

    const real root_half = std::sqrt(0.5);
    const nurbs_spline3<real> quarter_circle(
        {{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        2);
    check_point(quarter_circle.evaluate(0.5), {root_half, root_half, 0.0}, 1e-11,
                "rational quarter circle evaluation");
    check_near(quarter_circle.curvature(0.5), 1.0, 1e-11,
               "rational 3D unit circle has unit curvature");

    constexpr real narrow_domain_end = 1e-104;
    const nurbs_spline3<real> narrow_quarter_circle(
        quarter_circle.get_control_points(),
        quarter_circle.get_weights(),
        {0.0,
         0.0,
         0.0,
         narrow_domain_end,
         narrow_domain_end,
         narrow_domain_end},
        2);
    check_near(
        narrow_quarter_circle.curvature(narrow_domain_end / 2.0),
        1.0,
        1e-11,
        "3D curvature remains stable on a narrow native parameter domain");

    constexpr real wide_domain_end = 1e104;
    const nurbs_spline3<real> wide_quarter_circle(
        quarter_circle.get_control_points(),
        quarter_circle.get_weights(),
        {0.0,
         0.0,
         0.0,
         wide_domain_end,
         wide_domain_end,
         wide_domain_end},
        2);
    check_near(
        wide_quarter_circle.curvature(wide_domain_end / 2.0, 0.0),
        1.0,
        1e-11,
        "3D curvature remains stable on a wide native parameter domain");

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
