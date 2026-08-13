#include <nurbspath/config.hpp>

#include "test_support.hpp"

int main() {
    using namespace test_support;

    const plane3<real> plane(
        {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    const auto plane_result = nurbspath::distance_to_plane(
        point3<real>{2.0, 3.0, 4.0}, plane, 1e-10);
    check_near(plane_result.distance, 4.0, 1e-12, "point-plane distance");
    check_point(plane_result.closest_point, {2.0, 3.0, 0.0}, 1e-12,
                "point-plane closest point");

    const sphere3<real> sphere({0.0, 0.0, 0.0}, 2.0);
    const auto sphere_result = nurbspath::distance_to_sphere(
        point3<real>{5.0, 0.0, 0.0}, sphere, 1e-10);
    check_near(sphere_result.distance, 3.0, 1e-12, "point-sphere distance");
    check_point(sphere_result.closest_point, {2.0, 0.0, 0.0}, 1e-12,
                "point-sphere closest point");

    nurbspath::numerical_settings<real> settings;
    settings.residual_tolerance = 1e-9;
    settings.parameter_tolerance = 1e-12;
    settings.sample_count = 256;
    const auto line = make_line({-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    const auto spline_result = nurbspath::distance_to_spline(
        point3<real>{0.0, 3.0, 0.0}, line, settings);
    check_near(spline_result.distance, 3.0, 1e-8, "point-spline distance");
    check_near(spline_result.s, 0.5, 1e-7, "point-spline closest s");
    check_near(nurbspath::distance(
                   point3<real>{0.0, 0.0, 5e-11}, plane, 1e-9),
               0.0, 0.0, "distance tolerance snaps to zero");

    return finish("07_test_distances");
}
