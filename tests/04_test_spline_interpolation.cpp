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

    const std::vector<point3<real>> closed_samples{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.5},
        {-1.0, 0.0, 1.0},
        {0.0, -1.0, 0.5},
        {1.0, 0.0, 0.0}};
    const std::vector<real> closed_stations{2.0, 3.0, 4.0, 5.0, 6.0};
    const auto closed = nurbs_spline3<real>::interpolate(
        closed_samples, closed_stations, 3, true);
    check(closed.is_closed(), "closed 3D interpolation records closure");
    check_near(closed.s_min(), 2.0, 1e-12,
               "closed interpolation retains its native lower station");
    check_near(closed.s_max(), 6.0, 1e-12,
               "closed interpolation retains its native upper station");
    for (std::size_t index = 0; index < closed_samples.size(); ++index) {
        check_point(closed.evaluate(closed_stations[index]),
                    closed_samples[index], 2e-9,
                    "closed 3D interpolation passes through every sample");
    }
    check_point(closed.get_start(), closed.get_end(), 1e-12,
                "closed 3D cached seam points coincide");
    return finish("04_test_spline_interpolation");
}
