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
    check_near(line.approximate_arc_length(32), std::sqrt(52.0), 1e-11,
               "2D line approximate arc length");
    check(!line.is_closed() && !line.is_periodic(),
          "ordinary 2D spline is open and nonperiodic");
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

    const nurbs_spline2<real> closed_polyline(
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
        1,
        true,
        std::size_t(1));
    check(closed_polyline.is_closed() && !closed_polyline.is_periodic(),
          "closed 2D constructor records a nonperiodic seam");
    check_point2(closed_polyline.get_start(), closed_polyline.get_end(), 0.0,
                 "closed 2D cached endpoints coincide");

    const nurbs_spline2<real> periodic_polyline(
        {{1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {1.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {-1.0, 0.0, 1.0, 2.0, 3.0, 4.0},
        1,
        true,
        std::size_t(3));
    check(periodic_polyline.is_closed() && periodic_polyline.is_periodic(),
          "periodic 2D constructor retains independent closure");
    check(periodic_polyline.period_count() == 3,
          "periodic 2D constructor retains its finite period count");
    check_point2(periodic_polyline.evaluate(3.5),
                 periodic_polyline.evaluate(0.5), 1e-12,
                 "closed 2D repetition advances after one period");
    check_near(periodic_polyline.s_max(), 9.0, 1e-12,
               "finite 2D repetition extends the active domain");

    const nurbs_spline2<real> translated_polyline(
        {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {-1.0, 0.0, 1.0, 2.0, 3.0, 4.0},
        1,
        false,
        std::size_t(3));
    check_point2(
        translated_polyline.evaluate(3.5),
        translated_polyline.evaluate(0.5) + vector2<real>{3.0, 0.0},
        1e-12,
        "open 2D repetitions join by endpoint translation");
    check_point2(translated_polyline.get_end(), {9.0, 0.0}, 1e-12,
                 "finite repeated 2D endpoint includes all periods");

    const nurbs_spline2<real> unlimited_polyline(
        {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {-1.0, 0.0, 1.0, 2.0, 3.0, 4.0},
        1,
        false,
        std::size_t(0));
    check(unlimited_polyline.is_periodic() &&
              unlimited_polyline.period_count() == 0 &&
              std::isinf(unlimited_polyline.s_max()),
          "zero period count creates an unlimited forward 2D path");
    check_point2(
        unlimited_polyline.evaluate(30.5),
        unlimited_polyline.evaluate(0.5) + vector2<real>{30.0, 0.0},
        1e-11,
        "unlimited 2D path evaluates translated periods");
    bool unlimited_has_no_end = false;
    try {
        static_cast<void>(unlimited_polyline.get_end());
    } catch (const std::domain_error&) {
        unlimited_has_no_end = true;
    }
    check(unlimited_has_no_end, "unlimited 2D path reports that it has no end");
    bool unlimited_has_no_total_length = false;
    try {
        static_cast<void>(unlimited_polyline.approximate_arc_length());
    } catch (const std::domain_error&) {
        unlimited_has_no_total_length = true;
    }
    check(unlimited_has_no_total_length,
          "unlimited 2D path has no finite total arc length");

    bool rejected_finite_overrun = false;
    try {
        static_cast<void>(translated_polyline.evaluate(9.1));
    } catch (const std::out_of_range&) {
        rejected_finite_overrun = true;
    }
    check(rejected_finite_overrun,
          "finite repeated 2D path rejects parameters after its final period");

    const std::vector<point2<real>> periodic_samples{
        {1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}, {1.0, 0.0}};
    const std::vector<real> periodic_stations{0.0, 1.0, 2.0, 3.0, 4.0};
    const auto closed_interpolated = nurbs_spline2<real>::interpolate(
        periodic_samples, periodic_stations, 2, true, std::size_t(1));
    check(closed_interpolated.is_closed() &&
              !closed_interpolated.is_periodic(),
          "closed 2D interpolation retains clamped nonperiodic behavior");
    const auto periodic = nurbs_spline2<real>::interpolate(
        periodic_samples, periodic_stations, 3, true, std::size_t(3));
    check(periodic.is_closed() && periodic.is_periodic(),
          "periodic 2D interpolation publishes closure and repetition");
    for (std::size_t index = 0; index < periodic_samples.size(); ++index) {
        check_point2(periodic.evaluate(periodic_stations[index]),
                     periodic_samples[index], 1e-9,
                     "periodic 2D interpolation passes through cyclic sample");
    }
    check_point2(periodic.get_start(), periodic.get_end(), 1e-12,
                 "periodic 2D cached seam points coincide");
    check(periodic.first_derivative(periodic.s_min()).approximately_equal(
              periodic.first_derivative(periodic.s_max()), 1e-9),
          "periodic 2D first derivative is continuous at seam");
    check(periodic.second_derivative(periodic.s_min()).approximately_equal(
              periodic.second_derivative(periodic.s_max()), 1e-9),
          "periodic 2D second derivative is continuous at seam");

    interpolated.adopt_to_points(
        periodic_samples, periodic_stations, 3, true, std::size_t(2));
    check(interpolated.is_periodic() && interpolated.period_count() == 2 &&
              interpolated.get_start().approximately_equal(
                  interpolated.get_end(), 1e-12),
          "2D adopt_to_points accepts closure and a period count");

    bool rejected_open_closed_flag = false;
    try {
        static_cast<void>(nurbs_spline2<real>(
            {{0.0, 0.0}, {1.0, 0.0}},
            {1.0, 1.0},
            {0.0, 0.0, 1.0, 1.0},
            1,
            true,
            std::size_t(1)));
    } catch (const std::invalid_argument&) {
        rejected_open_closed_flag = true;
    }
    check(rejected_open_closed_flag,
          "closed 2D constructor rejects an open seam");

    bool rejected_invalid_periodic_definition = false;
    try {
        static_cast<void>(nurbs_spline2<real>(
            {{1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {1.0, 0.0}},
            {1.0, 1.0, 1.0, 2.0},
            {-1.0, 0.0, 1.0, 2.0, 3.0, 4.0},
            1,
            true,
            std::size_t(2)));
    } catch (const std::invalid_argument&) {
        rejected_invalid_periodic_definition = true;
    }
    check(rejected_invalid_periodic_definition,
          "periodic 2D constructor validates repeated seam weights");

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
    return finish("11_test_2d_spline");
}
