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
    return finish("11_test_2d_spline");
}
