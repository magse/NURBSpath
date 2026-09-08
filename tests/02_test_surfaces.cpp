#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

int main() {
    using namespace test_support;

    const sphere3<real> sphere({1.0, 2.0, 3.0}, 2.0);
    const point3<real> on_sphere = sphere.point_at(0.0, 0.0);
    check_point(on_sphere, {3.0, 2.0, 3.0}, 1e-12, "sphere evaluation");
    const auto [sphere_u, sphere_v] = sphere.parameters_of(on_sphere);
    check_near(sphere_u, 0.0, 1e-12, "sphere u parameter");
    check_near(sphere_v, 0.0, 1e-12, "sphere v parameter");
    static_assert(std::is_constructible_v<vector3<real>, const sphere3<real>&>);
    static_assert(std::is_nothrow_constructible_v<
                  vector3<real>, const sphere3<real>&>);
    static_assert(!std::is_convertible_v<const sphere3<real>&, vector3<real>>);
    static_assert(!std::is_constructible_v<
                  vector3<real>, const sphere3<float>&>);
    static_assert(std::is_nothrow_assignable_v<
                  sphere3<real>&, const vector3<real>&>);
    static_assert(!std::is_assignable_v<
                  sphere3<real>&, const vector3<float>&>);
    static_assert(std::is_same_v<
                  decltype(std::declval<sphere3<real>&>() =
                           std::declval<const vector3<real>&>()),
                  sphere3<real>&>);
    static_assert(std::is_copy_assignable_v<sphere3<real>>);
    check(vector3<real>(sphere) == vector3<real>{1.0, 2.0, 3.0},
          "vector construction from a sphere uses its center");
    sphere3<real> moved_sphere({-1.0, -2.0, -3.0}, 4.5);
    sphere3<real>& moved_sphere_result =
        moved_sphere = vector3<real>{5.0, 6.0, 7.0};
    check(&moved_sphere_result == &moved_sphere &&
              moved_sphere.center() == point3<real>{5.0, 6.0, 7.0},
          "sphere assignment from a vector replaces its center");
    check_near(moved_sphere.radius(), 4.5, 0.0,
               "sphere assignment from a vector preserves its radius");

    static_assert(noexcept(std::declval<const sphere3<real>&>().get_center()));
    static_assert(noexcept(std::declval<const sphere3<real>&>().get_radius()));
    static_assert(noexcept(std::declval<sphere3<real>&>().set_center(
        std::declval<const point3<real>&>())));
    static_assert(noexcept(std::declval<sphere3<real>&>().set_center(
        std::declval<const vector3<real>&>())));
    check(moved_sphere.get_center() == moved_sphere.center(),
          "sphere get_center aliases the center getter");
    check_near(moved_sphere.get_radius(), moved_sphere.radius(), 0.0,
               "sphere get_radius aliases the radius getter");

    moved_sphere.set_center({-4.0, -5.0, -6.0});
    check(moved_sphere.get_center() == point3<real>{-4.0, -5.0, -6.0} &&
              moved_sphere.get_radius() == 4.5,
          "sphere point center setter preserves radius");
    moved_sphere.set_center(vector3<real>{7.0, 8.0, 9.0});
    check(moved_sphere.get_center() == point3<real>{7.0, 8.0, 9.0} &&
              moved_sphere.get_radius() == 4.5,
          "sphere vector center setter preserves radius");

    moved_sphere.set_radius(2.25);
    check(moved_sphere.get_center() == point3<real>{7.0, 8.0, 9.0} &&
              moved_sphere.get_radius() == 2.25,
          "sphere radius setter preserves center");
    bool rejected_sphere_radius = false;
    try {
        moved_sphere.set_radius(0.0);
    } catch (const std::invalid_argument&) {
        rejected_sphere_radius = true;
    }
    check(rejected_sphere_radius &&
              moved_sphere.get_center() == point3<real>{7.0, 8.0, 9.0} &&
              moved_sphere.get_radius() == 2.25,
          "sphere radius setter rejects zero without mutation");

    moved_sphere.set_center_and_radius({1.5, 2.5, 3.5}, 6.0);
    check(moved_sphere.get_center() == point3<real>{1.5, 2.5, 3.5} &&
              moved_sphere.get_radius() == 6.0,
          "sphere point center and radius setter updates both values");
    moved_sphere.set_center_and_radius(
        vector3<real>{-1.5, -2.5, -3.5}, 7.0);
    check(moved_sphere.get_center() == point3<real>{-1.5, -2.5, -3.5} &&
              moved_sphere.get_radius() == 7.0,
          "sphere vector center and radius setter updates both values");
    check_point(
        moved_sphere.point_at(0.0, 0.0),
        {5.5, -2.5, -3.5},
        1e-12,
        "sphere evaluation uses the updated center and radius");
    rejected_sphere_radius = false;
    try {
        moved_sphere.set_center_and_radius(
            {100.0, 200.0, 300.0},
            std::numeric_limits<real>::quiet_NaN());
    } catch (const std::invalid_argument&) {
        rejected_sphere_radius = true;
    }
    check(rejected_sphere_radius &&
              moved_sphere.get_center() == point3<real>{-1.5, -2.5, -3.5} &&
              moved_sphere.get_radius() == 7.0,
          "sphere combined setter rejects invalid radius atomically");

    const plane3<real> plane(
        {1.0, 2.0, 3.0}, {0.0, 0.0, 2.0}, {1.0, 0.0, 0.0});
    check_near(plane.u_direction().dot(plane.v_direction()), 0.0, 1e-12,
               "plane axes are orthogonal");
    check_point(plane.point_at(2.0, -1.0), {3.0, 1.0, 3.0}, 1e-12,
                "plane evaluation");
    const auto [plane_u, plane_v] = plane.parameters_of({3.0, 1.0, 8.0});
    check_near(plane_u, 2.0, 1e-12, "plane u parameter");
    check_near(plane_v, -1.0, 1e-12, "plane v parameter");
    static_assert(std::is_constructible_v<vector3<real>, const plane3<real>&>);
    static_assert(std::is_nothrow_constructible_v<
                  vector3<real>, const plane3<real>&>);
    static_assert(!std::is_convertible_v<const plane3<real>&, vector3<real>>);
    static_assert(!std::is_constructible_v<
                  vector3<real>, const plane3<float>&>);
    check(vector3<real>(plane) == vector3<real>{1.0, 2.0, 3.0},
          "vector construction from a plane uses its parameter origin");

    static_assert(std::is_same_v<
                  decltype(std::declval<const plane3<real>&>().get_origin()),
                  const point3<real>&>);
    static_assert(std::is_same_v<
                  decltype(std::declval<const plane3<real>&>().get_normal()),
                  const vector3<real>&>);
    static_assert(noexcept(std::declval<const plane3<real>&>().get_origin()));
    static_assert(noexcept(std::declval<const plane3<real>&>().get_normal()));
    static_assert(noexcept(std::declval<plane3<real>&>().set_origin(
        std::declval<const point3<real>&>())));
    static_assert(noexcept(std::declval<plane3<real>&>().set_origin(
        std::declval<const vector3<real>&>())));

    plane3<real> updated_plane(
        {1.0, 2.0, 3.0}, {0.0, 0.0, 2.0}, {1.0, 0.0, 0.0});
    check(&updated_plane.get_origin() == &updated_plane.origin() &&
              &updated_plane.get_normal() == &updated_plane.normal() &&
              &updated_plane.get_u_direction() ==
                  &updated_plane.u_direction() &&
              &updated_plane.get_v_direction() ==
                  &updated_plane.v_direction(),
          "plane get accessors agree with legacy accessors");

    updated_plane.set_origin({4.0, 5.0, 6.0});
    check(updated_plane.get_origin() == point3<real>{4.0, 5.0, 6.0} &&
              updated_plane.get_normal() == vector3<real>{0.0, 0.0, 1.0} &&
              updated_plane.get_u_direction() ==
                  vector3<real>{1.0, 0.0, 0.0},
          "plane point origin setter preserves the coordinate frame");
    updated_plane.set_origin(vector3<real>{7.0, 8.0, 9.0});
    check(updated_plane.get_origin() == point3<real>{7.0, 8.0, 9.0},
          "plane vector origin setter copies vector components");

    updated_plane.set_normal({0.0, 2.0, 0.0});
    check(updated_plane.get_origin() == point3<real>{7.0, 8.0, 9.0} &&
              updated_plane.get_normal().approximately_equal(
                  {0.0, 1.0, 0.0}, 1e-12) &&
              updated_plane.get_u_direction().approximately_equal(
                  {0.0, 0.0, 1.0}, 1e-12) &&
              updated_plane.get_v_direction().approximately_equal(
                  {1.0, 0.0, 0.0}, 1e-12),
          "plane normal setter preserves origin and regenerates its frame");

    updated_plane.set_u_direction({2.0, 5.0, 0.0});
    check(updated_plane.get_u_direction().approximately_equal(
              {1.0, 0.0, 0.0}, 1e-12) &&
              updated_plane.get_v_direction().approximately_equal(
                  {0.0, 0.0, -1.0}, 1e-12) &&
              updated_plane.get_u_direction()
                      .cross(updated_plane.get_v_direction())
                      .approximately_equal(updated_plane.get_normal(), 1e-12),
          "plane u setter projects its hint and preserves a right-handed frame");

    const point3<real> frame_origin_before = updated_plane.get_origin();
    const vector3<real> frame_normal_before = updated_plane.get_normal();
    const vector3<real> frame_u_before = updated_plane.get_u_direction();
    const vector3<real> frame_v_before = updated_plane.get_v_direction();
    bool rejected_plane_frame = false;
    try {
        updated_plane.set_u_direction(updated_plane.get_normal());
    } catch (const std::domain_error&) {
        rejected_plane_frame = true;
    }
    check(rejected_plane_frame &&
              updated_plane.get_origin() == frame_origin_before &&
              updated_plane.get_normal() == frame_normal_before &&
              updated_plane.get_u_direction() == frame_u_before &&
              updated_plane.get_v_direction() == frame_v_before,
          "plane frame setter rejects a normal-parallel u hint atomically");

    updated_plane.set_definition(
        {1.0, -2.0, 3.0}, {0.0, 0.0, 4.0}, {2.0, 0.0, 0.0});
    check(updated_plane.get_origin() == point3<real>{1.0, -2.0, 3.0} &&
              updated_plane.get_normal().approximately_equal(
                  {0.0, 0.0, 1.0}, 1e-12) &&
              updated_plane.get_u_direction().approximately_equal(
                  {1.0, 0.0, 0.0}, 1e-12),
          "plane complete setter accepts a point origin and oriented frame");
    updated_plane.set_definition(
        vector3<real>{-4.0, 5.0, -6.0},
        {0.0, 0.0, -3.0},
        {0.0, 2.0, 0.0});
    check(updated_plane.get_origin() == point3<real>{-4.0, 5.0, -6.0} &&
              updated_plane.get_normal().approximately_equal(
                  {0.0, 0.0, -1.0}, 1e-12) &&
              updated_plane.get_u_direction().approximately_equal(
                  {0.0, 1.0, 0.0}, 1e-12),
          "plane complete setter accepts a vector origin");
    check_point(updated_plane.point_at(2.0, 3.0), {-1.0, 7.0, -6.0}, 1e-12,
                "plane evaluation uses the updated oriented definition");

    updated_plane.set_definition({0.0, 0.0, 2.0}, 4.0);
    check(updated_plane.get_origin().approximately_equal(
              {0.0, 0.0, 4.0}, 1e-12) &&
              updated_plane.get_normal().approximately_equal(
                  {0.0, 0.0, 1.0}, 1e-12) &&
              updated_plane.get_signed_distance_from_origin() ==
                  updated_plane.signed_distance_from_origin(),
          "plane Hessian setter and get-prefixed distance accessor agree");

    const point3<real> definition_origin_before = updated_plane.get_origin();
    const vector3<real> definition_normal_before = updated_plane.get_normal();
    rejected_plane_frame = false;
    try {
        updated_plane.set_definition(
            {100.0, 200.0, 300.0},
            {0.0, 0.0, 0.0});
    } catch (const std::domain_error&) {
        rejected_plane_frame = true;
    }
    check(rejected_plane_frame &&
              updated_plane.get_origin() == definition_origin_before &&
              updated_plane.get_normal() == definition_normal_before,
          "plane complete setter validates before changing its definition");

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
