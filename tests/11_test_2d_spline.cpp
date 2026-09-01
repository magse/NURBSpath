#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

int main() {
    using namespace test_support;

    const auto line = make_line2({1.0, -2.0}, {5.0, 4.0}, 2.0, 6.0);
    check_point2(line.evaluate(4.0), {3.0, 1.0}, 1e-12,
                 "2D line spline evaluation");
    check(line.first_derivative(4.0).approximately_equal({1.0, 1.5}, 1e-12),
          "2D line analytic first derivative");
    check(line.second_derivative(4.0).is_near_zero(1e-12),
          "2D line analytic second derivative");
    check(line.third_derivative(4.0).is_near_zero(1e-12),
          "unit-weight 2D line analytic third derivative");
    check_near(line.approximate_arc_length(32), std::sqrt(52.0), 1e-11,
               "2D line approximate arc length");
    check(!line.is_closed(), "ordinary 2D spline is open");
    check_point2(line.get_start(), line.evaluate(line.s_min()), 0.0,
                 "2D start getter returns cached domain start");
    check_point2(line.get_end(), line.evaluate(line.s_max()), 0.0,
                 "2D end getter returns cached domain end");

    const std::vector<point2<real>> samples{
        {0.0, 0.0}, {1.0, 0.4}, {2.0, 1.2}, {3.0, 1.0}, {4.0, 0.0}};
    const std::vector<real> stations{5.0, 6.1, 7.5, 9.0, 11.0};
    auto interpolated = nurbs_spline2<real>::interpolate(samples, stations, 3);
    check_near(interpolated.s_min(), stations.front(), 1e-12,
               "2D interpolation preserves lower station");
    check_near(interpolated.s_max(), stations.back(), 1e-12,
               "2D interpolation preserves upper station");
    for (std::size_t index = 0; index < samples.size(); ++index) {
        check_point2(interpolated.evaluate(stations[index]), samples[index], 1e-9,
                     "2D global interpolation passes through sample");
    }

    const real root_half = std::sqrt(0.5);
    const nurbs_spline2<real> quarter_circle(
        {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        2);
    const point2<real> middle = quarter_circle.evaluate(0.5);
    check_point2(middle, {root_half, root_half}, 1e-12,
                 "rational 2D quarter circle evaluation");
    check_near(quarter_circle.tangent(0.5).dot(middle - point2<real>::origin()),
               0.0, 1e-12,
               "2D circle spline tangent is radial-orthogonal");

    const nurbs_spline2<real> unit_cubic(
        {{0.0, 0.0}, {1.0, 0.0}, {1.0, 2.0}, {4.0, 3.0}},
        {1.0, 1.0, 1.0, 1.0},
        {2.0, 2.0, 2.0, 2.0, 4.0, 4.0, 4.0, 4.0},
        3);
    const vector2<real> expected_cubic_third{3.0, -9.0 / 4.0};
    check(unit_cubic.third_derivative(2.0).approximately_equal(
              expected_cubic_third, 1e-12) &&
              unit_cubic.third_derivative(3.0).approximately_equal(
                  expected_cubic_third, 1e-12) &&
              unit_cubic.third_derivative(4.0).approximately_equal(
                  expected_cubic_third, 1e-12),
          "2D cubic third derivative respects native s and endpoint spans");

    const nurbs_spline2<real> two_span_cubic(
        {{0.0, 0.0},
         {0.0, 0.0},
         {0.0, 0.0},
         {1.0, 0.0},
         {1.0, 0.0},
         {1.0, 0.0},
         {1.0, 2.0}},
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0},
        3);
    check(two_span_cubic.third_derivative(0.25).approximately_equal(
              {48.0, 0.0}, 1e-12) &&
              two_span_cubic.third_derivative(0.5).approximately_equal(
                  {0.0, 96.0}, 1e-12) &&
              two_span_cubic.third_derivative(1.0).approximately_equal(
                  {0.0, 96.0}, 1e-12),
          "2D third derivative selects the right span at an internal knot");

    const nurbs_spline2<real> rational_line(
        {{0.0, 0.0}, {1.0, 2.0}},
        {1.0, 2.0},
        {0.0, 0.0, 1.0, 1.0},
        1);
    check(rational_line.second_derivative(0.5).approximately_equal(
              {-32.0 / 27.0, -64.0 / 27.0}, 1e-12),
          "degree-one rational 2D spline has an analytic second derivative");
    check(rational_line.third_derivative(0.5).approximately_equal(
              {64.0 / 27.0, 128.0 / 27.0}, 1e-12),
          "degree-one rational 2D spline has an analytic third derivative");

    const nurbs_spline2<real> rational_cubic(
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}},
        {1.0, 1.0, 1.0, 5.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0},
        3);
    check(rational_cubic.third_derivative(0.5).approximately_equal(
              {32.0, 8.0}, 1e-12),
          "rational 2D cubic third derivative uses all quotient-rule terms");

    const nurbs_spline2<real> closed_polyline(
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
        1,
        true);
    check(closed_polyline.is_closed(),
          "closed 2D constructor records its seam state");
    check_point2(closed_polyline.get_start(), closed_polyline.get_end(), 0.0,
                 "closed 2D cached endpoints coincide");

    const std::vector<point2<real>> closed_samples{
        {1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}, {1.0, 0.0}};
    const std::vector<real> closed_stations{0.0, 1.0, 2.0, 3.0, 4.0};
    const auto closed_interpolated = nurbs_spline2<real>::interpolate(
        closed_samples, closed_stations, 3, true);
    check(closed_interpolated.is_closed(),
          "closed 2D interpolation retains closure");
    for (std::size_t index = 0; index < closed_samples.size(); ++index) {
        check_point2(closed_interpolated.evaluate(closed_stations[index]),
                     closed_samples[index], 1e-9,
                     "closed 2D interpolation passes through every sample");
    }
    check_point2(closed_interpolated.get_start(), closed_interpolated.get_end(),
                 1e-12, "closed 2D cached seam points coincide");

    interpolated.adopt_to_points(
        closed_samples, closed_stations, 3, true);
    check(interpolated.is_closed() &&
              interpolated.get_start().approximately_equal(
                  interpolated.get_end(), 1e-12),
          "2D adopt_to_points accepts closure");

    bool rejected_open_closed_flag = false;
    try {
        static_cast<void>(nurbs_spline2<real>(
            {{0.0, 0.0}, {1.0, 0.0}},
            {1.0, 1.0},
            {0.0, 0.0, 1.0, 1.0},
            1,
            true));
    } catch (const std::invalid_argument&) {
        rejected_open_closed_flag = true;
    }
    check(rejected_open_closed_flag,
          "closed 2D constructor rejects an open seam");

    interpolated.adopt_to_points({{0.0, 0.0}, {2.0, 0.0}}, {10.0, 12.0}, 1);
    check_point2(interpolated.evaluate(11.0), {1.0, 0.0}, 1e-12,
                 "2D adopt_to_points replaces spline");

    bool rejected_parameter = false;
    try {
        static_cast<void>(line.evaluate(1.0));
    } catch (const std::out_of_range&) {
        rejected_parameter = true;
    }
    check(rejected_parameter, "2D spline rejects out-of-domain s");

    // These active boundaries sit inside runs of degree + 1 equal knots.
    // Outer knots make the layouts valid while exposing zero-width candidate
    // spans at s_min and s_max.
    const nurbs_spline2<real> repeated_start_boundary(
        {{-99.0, 0.0},
         {-9.0, 0.0},
         {0.0, 0.0},
         {0.25, 0.0},
         {0.75, 0.0},
         {1.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
        {-2.0, -1.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 2.0},
        2);
    const point2<real> repeated_start =
        repeated_start_boundary.evaluate(repeated_start_boundary.s_min());
    check(std::isfinite(repeated_start.x) && std::isfinite(repeated_start.y),
          "repeated 2D active start knot evaluates finitely");
    check_point2(repeated_start, {0.0, 0.0}, 1e-12,
                 "repeated 2D active start knot uses its right-hand span");
    check_point2(repeated_start_boundary.get_start(), repeated_start, 0.0,
                 "repeated 2D active start knot updates the cached start");
    check(repeated_start_boundary.first_derivative(0.0).approximately_equal(
              {1.0, 0.0}, 1e-12) &&
              repeated_start_boundary.second_derivative(0.0).is_near_zero(
                  1e-12) &&
              repeated_start_boundary.third_derivative(0.0).is_near_zero(
                  1e-12),
          "repeated 2D active start knot has finite right-hand derivatives");
    bool rejected_repeated_boundary_seam = false;
    try {
        static_cast<void>(nurbs_spline2<real>(
            repeated_start_boundary.get_control_points(),
            repeated_start_boundary.get_weights(),
            repeated_start_boundary.get_knots(),
            repeated_start_boundary.degree(),
            true));
    } catch (const std::invalid_argument&) {
        rejected_repeated_boundary_seam = true;
    }
    check(rejected_repeated_boundary_seam,
          "repeated 2D active boundary cannot hide an open seam");

    const nurbs_spline2<real> repeated_end_boundary(
        {{0.0, 0.0},
         {0.25, 0.0},
         {0.75, 0.0},
         {1.0, 0.0},
         {9.0, 0.0},
         {99.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
        {-1.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 2.0, 3.0},
        2);
    const point2<real> repeated_end =
        repeated_end_boundary.evaluate(repeated_end_boundary.s_max());
    check(std::isfinite(repeated_end.x) && std::isfinite(repeated_end.y),
          "repeated 2D active end knot evaluates finitely");
    check_point2(repeated_end, {1.0, 0.0}, 1e-12,
                 "repeated 2D active end knot uses its left-hand span");
    check_point2(repeated_end_boundary.get_end(), repeated_end, 0.0,
                 "repeated 2D active end knot updates the cached end");
    check(repeated_end_boundary.first_derivative(1.0).approximately_equal(
              {1.0, 0.0}, 1e-12) &&
              repeated_end_boundary.second_derivative(1.0).is_near_zero(
                  1e-12) &&
              repeated_end_boundary.third_derivative(1.0).is_near_zero(
                  1e-12),
          "repeated 2D active end knot has finite left-hand derivatives");
    return finish("11_test_2d_spline");
}
