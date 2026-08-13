#include <nurbspath/config.hpp>

#include "test_support.hpp"

int main() {
    using namespace test_support;

    const sphere3<real> sphere({1.0, 2.0, 3.0}, 2.0);
    const point3<real> on_sphere = sphere.point_at(0.0, 0.0);
    check_point(on_sphere, {3.0, 2.0, 3.0}, 1e-12, "sphere evaluation");
    const auto [sphere_u, sphere_v] = sphere.parameters_of(on_sphere);
    check_near(sphere_u, 0.0, 1e-12, "sphere u parameter");
    check_near(sphere_v, 0.0, 1e-12, "sphere v parameter");

    const plane3<real> plane(
        {1.0, 2.0, 3.0}, {0.0, 0.0, 2.0}, {1.0, 0.0, 0.0});
    check_near(plane.u_direction().dot(plane.v_direction()), 0.0, 1e-12,
               "plane axes are orthogonal");
    check_point(plane.point_at(2.0, -1.0), {3.0, 1.0, 3.0}, 1e-12,
                "plane evaluation");
    const auto [plane_u, plane_v] = plane.parameters_of({3.0, 1.0, 8.0});
    check_near(plane_u, 2.0, 1e-12, "plane u parameter");
    check_near(plane_v, -1.0, 1e-12, "plane v parameter");

    const plane3<real> hessian_plane(vector3<real>{0.0, 0.0, 4.0}, 3.0);
    check_point(hessian_plane.origin(), {0.0, 0.0, 3.0}, 1e-12,
                "normal-distance plane uses closest world-origin point");
    check_near(hessian_plane.signed_distance_from_origin(), 3.0, 1e-12,
               "normal-distance plane retains signed distance");
    check_near(hessian_plane.signed_distance_to({2.0, -5.0, 7.0}), 4.0, 1e-12,
               "normal-distance plane has expected equation");

    const vector3<real> arbitrary_normal{1.0, -2.0, 3.0};
    const plane3<real> arbitrary_plane(arbitrary_normal, -2.5);
    check(arbitrary_plane.normal().approximately_equal(
              arbitrary_normal.normalized(), 1e-12),
          "normal-distance plane accepts any normal direction");
    check_near(arbitrary_plane.signed_distance_from_origin(), -2.5, 1e-12,
               "normal-distance plane accepts negative placement");
    return finish("02_test_surfaces");
}
