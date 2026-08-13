#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <numbers>
#include <stdexcept>
#include <type_traits>

int main() {
    using namespace test_support;

    const vector2<real> first{3.0, 4.0};
    const vector2<real> second{-2.0, 1.0};
    check_near(first.length(), 5.0, 1e-12, "2D vector length");
    check_near(first.dot(second), -2.0, 1e-12, "2D dot product");
    check_near(first.cross(second), 11.0, 1e-12, "2D scalar cross product");
    check(first.perpendicular_left() == vector2<real>{-4.0, 3.0},
          "2D left perpendicular");
    check(first.perpendicular_right() == vector2<real>{4.0, -3.0},
          "2D right perpendicular");
    check(first.normalized().approximately_equal({0.6, 0.8}, 1e-12),
          "2D vector normalization");
    check_near(vector2<real>::unit_x().signed_angle_to(vector2<real>::unit_y()),
               std::numbers::pi_v<real> / 2.0, 1e-12,
               "2D signed angle");

    const point2<real> point{1.0, 2.0};
    check_point2(point + vector2<real>{2.0, -3.0}, {3.0, -1.0}, 1e-12,
                 "2D point translation");
    check((point2<real>{4.0, 6.0} - point).approximately_equal({3.0, 4.0}),
          "2D point difference is a vector");
    check_near(distance(point, point2<real>{4.0, 6.0}), 5.0, 1e-12,
               "2D point distance");

    const ray2<real> ray(point, {2.0, -1.0});
    check_point2(ray.evaluate(1.5), {4.0, 0.5}, 1e-12, "2D ray evaluation");
    check(ray.tangent().approximately_equal(
              vector2<real>{2.0, -1.0}.normalized(), 1e-12),
          "2D ray tangent");

    const circle2<real> circle({2.0, 3.0}, 2.0);
    check_point2(circle.point_at(0.0), {4.0, 3.0}, 1e-12,
                 "circle evaluation");
    check_near(circle.parameter_of({2.0, 1.0}),
               3.0 * std::numbers::pi_v<real> / 2.0, 1e-12,
               "circle angle normalization");
    check(circle.normal_at({2.0, 5.0}).approximately_equal({0.0, 1.0}),
          "circle outward normal");

    bool rejected_zero_ray = false;
    try {
        static_cast<void>(ray2<real>({0.0, 0.0}, {0.0, 0.0}));
    } catch (const std::invalid_argument&) {
        rejected_zero_ray = true;
    }
    check(rejected_zero_ray, "2D ray rejects zero direction");

    bool rejected_radius = false;
    try {
        static_cast<void>(circle2<real>({0.0, 0.0}, 0.0));
    } catch (const std::invalid_argument&) {
        rejected_radius = true;
    }
    check(rejected_radius, "circle rejects nonpositive radius");

    static_assert(!std::is_convertible_v<point2<real>, point3<real>>);
    static_assert(!std::is_convertible_v<vector2<real>, vector3<real>>);
    static_assert(!std::is_convertible_v<ray2<real>, ray3<real>>);
    return finish("10_test_2d_entities");
}
