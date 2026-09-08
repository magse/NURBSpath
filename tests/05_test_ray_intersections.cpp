#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <stdexcept>
#include <type_traits>
#include <utility>

int main() {
    using namespace test_support;

    nurbspath::numerical_settings<real> settings;
    settings.residual_tolerance = 1e-10;
    settings.parameter_tolerance = 1e-12;
    settings.sample_count = 512;

    const ray3<real> ray({-3.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    static_assert(std::is_constructible_v<vector3<real>, const ray3<real>&>);
    static_assert(std::is_nothrow_constructible_v<
                  vector3<real>, const ray3<real>&>);
    static_assert(!std::is_convertible_v<const ray3<real>&, vector3<real>>);
    static_assert(!std::is_constructible_v<
                  vector3<real>, const ray3<float>&>);
    check(vector3<real>(ray) == vector3<real>{-3.0, 0.0, 0.0},
          "vector construction from a ray uses its origin");

    static_assert(noexcept(std::declval<const ray3<real>&>().get_origin()));
    static_assert(noexcept(std::declval<const ray3<real>&>().get_direction()));
    static_assert(noexcept(std::declval<ray3<real>&>().set_origin(
        std::declval<const point3<real>&>())));
    static_assert(noexcept(std::declval<ray3<real>&>().set_origin(
        std::declval<const vector3<real>&>())));
    check(ray.get_origin() == ray.origin(),
          "ray get_origin aliases the origin getter");
    check(ray.get_direction() == ray.direction(),
          "ray get_direction aliases the direction getter");

    ray3<real> updated_ray({1.0, 2.0, 3.0}, {4.0, 0.0, 0.0});
    updated_ray.set_origin({-1.0, -2.0, -3.0});
    check(updated_ray.get_origin() == point3<real>{-1.0, -2.0, -3.0} &&
              updated_ray.get_direction() == vector3<real>{4.0, 0.0, 0.0},
          "ray point origin setter preserves direction");
    updated_ray.set_origin(vector3<real>{5.0, 6.0, 7.0});
    check(updated_ray.get_origin() == point3<real>{5.0, 6.0, 7.0} &&
              updated_ray.get_direction() == vector3<real>{4.0, 0.0, 0.0},
          "ray vector origin setter preserves direction");

    updated_ray.set_direction({0.0, -3.0, 0.0});
    check(updated_ray.get_origin() == point3<real>{5.0, 6.0, 7.0} &&
              updated_ray.get_direction() == vector3<real>{0.0, -3.0, 0.0},
          "ray direction setter preserves origin and does not normalize");
    check_point(updated_ray.point_at(2.0), {5.0, 0.0, 7.0}, 1e-12,
                "ray evaluation uses updated origin and direction");

    bool rejected_direction = false;
    try {
        updated_ray.set_direction({0.0, 0.0, 0.0});
    } catch (const std::invalid_argument&) {
        rejected_direction = true;
    }
    check(rejected_direction &&
              updated_ray.get_origin() == point3<real>{5.0, 6.0, 7.0} &&
              updated_ray.get_direction() == vector3<real>{0.0, -3.0, 0.0},
          "ray direction setter rejects zero without mutation");

    updated_ray.set_origin_and_direction(
        {-4.0, -5.0, -6.0}, {2.0, 4.0, 6.0});
    check(updated_ray.get_origin() == point3<real>{-4.0, -5.0, -6.0} &&
              updated_ray.get_direction() == vector3<real>{2.0, 4.0, 6.0},
          "ray point origin and direction setter updates both values");
    updated_ray.set_origin_and_direction(
        vector3<real>{8.0, 9.0, 10.0}, {-1.0, -2.0, -3.0});
    check(updated_ray.get_origin() == point3<real>{8.0, 9.0, 10.0} &&
              updated_ray.get_direction() == vector3<real>{-1.0, -2.0, -3.0},
          "ray vector origin and direction setter updates both values");

    rejected_direction = false;
    try {
        updated_ray.set_origin_and_direction(
            {100.0, 200.0, 300.0}, {1e-4, 0.0, 0.0}, 1e-3);
    } catch (const std::invalid_argument&) {
        rejected_direction = true;
    }
    check(rejected_direction &&
              updated_ray.get_origin() == point3<real>{8.0, 9.0, 10.0} &&
              updated_ray.get_direction() == vector3<real>{-1.0, -2.0, -3.0},
          "ray combined setter validates direction before changing either value");

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
