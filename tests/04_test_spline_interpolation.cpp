#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <cmath>
#include <vector>

int main() {
    using namespace test_support;

    const std::vector<point3<real>> samples{
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 0.5},
        {2.0, 4.0, 1.0},
        {3.0, 9.0, 1.5},
        {4.0, 16.0, 2.0}
    };
    const std::vector<real> stations{0.0, 1.2, 2.8, 5.0, 8.0};
    const auto spline = nurbs_spline3<real>::interpolate(samples, stations, 3);
    check_near(spline.s_min(), stations.front(), 1e-12,
               "interpolation keeps s minimum");
    check_near(spline.s_max(), stations.back(), 1e-12,
               "interpolation keeps s maximum");
    for (std::size_t index = 0; index < samples.size(); ++index) {
        check_point(spline.evaluate(stations[index]), samples[index], 2e-9,
                    "global interpolation passes through sample");
    }
    check(std::isfinite(spline.first_derivative(2.0).length()),
          "interpolated first derivative is finite");
    check(std::isfinite(spline.second_derivative(2.0).length()),
          "interpolated second derivative is finite");

    const std::vector<point3<real>> periodic_samples{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.5},
        {-1.0, 0.0, 1.0},
        {0.0, -1.0, 0.5},
        {1.0, 0.0, 0.0}};
    const std::vector<real> periodic_stations{2.0, 3.0, 4.0, 5.0, 6.0};
    const auto periodic = nurbs_spline3<real>::interpolate(
        periodic_samples, periodic_stations, 3, true, std::size_t(3));
    check(periodic.is_closed() && periodic.is_periodic(),
          "periodic 3D interpolation publishes closure and repetition");
    check(periodic.period_count() == 3,
          "periodic 3D interpolation retains its finite period count");
    check_near(periodic.s_min(), 2.0, 1e-12,
               "periodic interpolation retains its native lower station");
    check_near(periodic.period_s_max(), 6.0, 1e-12,
               "periodic interpolation retains its fundamental upper station");
    check_near(periodic.s_max(), 14.0, 1e-12,
               "finite repetition extends the 3D active domain");
    for (std::size_t index = 0; index < periodic_samples.size(); ++index) {
        check_point(periodic.evaluate(periodic_stations[index]),
                    periodic_samples[index], 2e-9,
                    "periodic 3D interpolation passes through cyclic sample");
    }
    check_point(periodic.get_start(), periodic.get_end(), 1e-12,
                "periodic 3D cached seam points coincide");
    check_point(periodic.evaluate(6.5), periodic.evaluate(2.5), 1e-10,
                "closed 3D repetition advances by one period");
    check(periodic.first_derivative(periodic.s_min()).approximately_equal(
              periodic.first_derivative(periodic.s_max()), 1e-9),
          "periodic 3D first derivative is continuous at seam");
    check(periodic.second_derivative(periodic.s_min()).approximately_equal(
              periodic.second_derivative(periodic.s_max()), 1e-9),
          "periodic 3D second derivative is continuous at seam");

    const std::vector<point3<real>> open_period_samples{
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 0.0},
        {2.0, 0.0, 1.0},
        {3.0, -1.0, 1.0},
        {4.0, 0.0, 2.0}};
    const std::vector<real> open_period_stations{0.0, 1.0, 2.0, 3.0, 4.0};
    const auto repeated = nurbs_spline3<real>::interpolate(
        open_period_samples,
        open_period_stations,
        3,
        false,
        std::size_t(3));
    check(!repeated.is_closed() && repeated.is_periodic(),
          "open repeated 3D interpolation keeps closure independent");
    check_point(
        repeated.evaluate(5.5),
        repeated.evaluate(1.5) + vector3<real>{4.0, 0.0, 2.0},
        2e-9,
        "next 3D period starts at the translated previous endpoint");
    check_point(repeated.get_end(), {12.0, 0.0, 6.0}, 2e-9,
                "finite repeated 3D endpoint includes every displacement");
    return finish("04_test_spline_interpolation");
}
