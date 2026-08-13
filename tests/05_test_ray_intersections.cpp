#include <nurbspath/config.hpp>

#include "test_support.hpp"

int main() {
    using namespace test_support;

    nurbspath::numerical_settings<real> settings;
    settings.residual_tolerance = 1e-10;
    settings.parameter_tolerance = 1e-12;
    settings.sample_count = 512;

    const ray3<real> ray({-3.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    const sphere3<real> sphere({0.0, 0.0, 0.0}, 1.0);
    const auto sphere_hits = nurbspath::intersect_ray_sphere(ray, sphere, settings);
    check(sphere_hits.kind == nurbspath::intersection_kind::discrete,
          "ray-sphere intersection kind");
    check(sphere_hits.points.size() == 2, "ray-sphere has two intersections");
    if (sphere_hits.points.size() == 2) {
        check_near(sphere_hits.points[0].s, 2.0, 1e-8, "ray-sphere first s");
        check_near(sphere_hits.points[1].s, 4.0, 1e-8, "ray-sphere second s");
        check(sphere_hits.points[0].s < sphere_hits.points[1].s,
              "ray-sphere contacts are ordered");
    }
    const ray3<real> missing_ray({-3.0, 2.0, 0.0}, {1.0, 0.0, 0.0});
    check(!nurbspath::intersect_ray_sphere(missing_ray, sphere, settings)
               .has_intersection(),
          "ray-sphere no contact");

    const plane3<real> plane({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    const ray3<real> descending({2.0, 3.0, 5.0}, {0.0, 0.0, -2.0});
    const auto plane_hit = nurbspath::intersect_ray_plane(descending, plane, settings);
    check(plane_hit.points.size() == 1, "ray-plane has one intersection");
    if (!plane_hit.points.empty()) {
        check_near(plane_hit.points[0].s, 2.5, 1e-8, "ray-plane s");
    }
    const ray3<real> away({0.0, 0.0, 1.0}, {0.0, 0.0, 1.0});
    check(!nurbspath::intersect_ray_plane(away, plane, settings).has_intersection(),
          "ray-plane contact behind ray is rejected");
    const ray3<real> in_plane({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    check(nurbspath::intersect_ray_plane(in_plane, plane, settings).is_coincident(),
          "coplanar ray is coincident");
    return finish("05_test_ray_intersections");
}
