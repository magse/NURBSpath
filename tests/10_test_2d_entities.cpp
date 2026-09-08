#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

int main() {
    using namespace test_support;

    static_assert(std::is_aggregate_v<point2<real>>);
    static_assert(noexcept(point2<real>{}.magnitude()));
    static_assert(noexcept(point2<real>{}.manhattan_distance()));
    static_assert(std::is_same_v<
                  decltype(point2<real>{}.manhattan_distance()), real>);
    point2<real> point_components{};
    check(point_components.x == 0.0 && point_components.y == 0.0,
          "default 2D point coordinates are zero");
    point_components.x = 2.0;
    point_components.y = -3.0;
    check(point_components == point2<real>{2.0, -3.0},
          "2D point coordinates are public and mutable");
    const point2<real> magnitude_point{.x = -3.0, .y = 4.0};
    check_near(magnitude_point.magnitude(), 5.0, 1e-12,
               "2D point magnitude is distance from 2D origin");
    check_near(magnitude_point.manhattan_distance(), 7.0, 1e-12,
               "2D point Manhattan distance is L1 distance from 2D origin");
    check(point2<real>::origin().manhattan_distance() == 0.0,
          "2D origin has zero Manhattan distance");

    static_assert(!std::is_aggregate_v<vector2<real>>);
    static_assert(std::is_nothrow_default_constructible_v<vector2<real>>);
    static_assert(
        std::is_nothrow_constructible_v<vector2<real>, real, real>);
    static_assert(std::is_constructible_v<
                  vector2<real>, const point2<real>&>);
    static_assert(std::is_constructible_v<
                  vector2<real>, const circle2<real>&>);
    static_assert(std::is_constructible_v<
                  vector2<real>, const ray2<real>&>);
    static_assert(!std::is_convertible_v<point2<real>, vector2<real>>);
    static_assert(!std::is_convertible_v<circle2<real>, vector2<real>>);
    static_assert(!std::is_convertible_v<ray2<real>, vector2<real>>);
    static_assert(std::is_nothrow_constructible_v<
                  vector2<real>, const point2<real>&>);
    static_assert(std::is_nothrow_constructible_v<
                  vector2<real>, const circle2<real>&>);
    static_assert(std::is_nothrow_constructible_v<
                  vector2<real>, const ray2<real>&>);
    static_assert(!std::is_constructible_v<
                  vector2<real>, const point2<float>&>);
    static_assert(!std::is_constructible_v<
                  vector2<real>, const circle2<float>&>);
    static_assert(!std::is_constructible_v<
                  vector2<real>, const ray2<float>&>);
    static_assert(std::is_assignable_v<
                  point2<real>&, const vector2<real>&>);
    static_assert(std::is_nothrow_assignable_v<
                  point2<real>&, const vector2<real>&>);
    static_assert(!std::is_assignable_v<
                  point2<real>&, const vector2<float>&>);
    static_assert(std::is_same_v<
                  decltype(std::declval<point2<real>&>() =
                           std::declval<const vector2<real>&>()),
                  point2<real>&>);
    static_assert(std::is_assignable_v<
                  circle2<real>&, const vector2<real>&>);
    static_assert(std::is_nothrow_assignable_v<
                  circle2<real>&, const vector2<real>&>);
    static_assert(!std::is_assignable_v<
                  circle2<real>&, const vector2<float>&>);
    static_assert(std::is_same_v<
                  decltype(std::declval<circle2<real>&>() =
                           std::declval<const vector2<real>&>()),
                  circle2<real>&>);
    static_assert(std::is_copy_assignable_v<point2<real>>);
    static_assert(std::is_copy_assignable_v<circle2<real>>);
    static_assert([] {
        constexpr point2<real> source{1.25, -2.5};
        constexpr vector2<real> constructed(source);
        point2<real> assigned{};
        assigned = constructed;
        return constructed == vector2<real>{1.25, -2.5} &&
               assigned == source;
    }());
    vector2<real> components{};
    check(components.x == 0.0 && components.y == 0.0,
          "default 2D vector components are zero");
    components.x = 2.0;
    components.y = -3.0;
    check(components == vector2<real>{2.0, -3.0},
          "2D vector components are public and mutable");

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
    vector2<real> in_place{-3.0, 4.0};
    static_assert(std::is_void_v<decltype(in_place.normalize())>);
    in_place.normalize();
    check(in_place.approximately_equal({-0.6, 0.8}, 1e-12),
          "2D vector in-place normalization preserves direction");
    check_near(in_place.length(), 1.0, 1e-12,
               "2D in-place normalized vector has unit length");
    vector2<real> zero_vector{};
    bool rejected_zero_normalization = false;
    try {
        zero_vector.normalize();
    } catch (const std::domain_error&) {
        rejected_zero_normalization = true;
    }
    check(rejected_zero_normalization && zero_vector == vector2<real>{},
          "2D in-place normalization rejects and preserves zero vector");
    const real large_component = std::numeric_limits<real>::max() / 2.0;
    vector2<real> large_vector{large_component, -large_component};
    large_vector.normalize();
    check_near(large_vector.length(), 1.0, 1e-12,
               "2D in-place normalization handles large finite components");
    check_near(vector2<real>::unit_x().signed_angle_to(vector2<real>::unit_y()),
               std::numbers::pi_v<real> / 2.0, 1e-12,
               "2D signed angle");

    const point2<real> point{.x = 1.0, .y = 2.0};
    const point2<real> vector_source_point{-6.0, 7.5};
    const vector2<real> vector_from_point{vector_source_point};
    check(vector_from_point == vector2<real>{-6.0, 7.5} &&
              vector_source_point == point2<real>{-6.0, 7.5},
          "2D vector construction copies a point without changing it");
    const vector2<real> point_assignment_source{8.0, -9.0};
    point2<real> assigned_point{1.0, 2.0};
    point2<real>* assigned_point_result =
        &(assigned_point = point_assignment_source);
    check(assigned_point_result == &assigned_point &&
              assigned_point == point2<real>{8.0, -9.0} &&
              point_assignment_source == vector2<real>{8.0, -9.0},
          "2D point assignment copies vector components and returns itself");
    point2<real> brace_assigned_point{};
    brace_assigned_point = {4.0, -5.0};
    check(brace_assigned_point == point2<real>{4.0, -5.0},
          "2D point brace-list assignment remains unambiguous");
    check_point2(point + vector2<real>{2.0, -3.0}, {3.0, -1.0}, 1e-12,
                 "2D point translation");
    check((point2<real>{4.0, 6.0} - point).approximately_equal({3.0, 4.0}),
          "2D point difference is a vector");
    check_near(distance(point, point2<real>{4.0, 6.0}), 5.0, 1e-12,
               "2D point distance");

    std::ostringstream output;
    output << point2<real>{1.25, -2.5} << '|'
           << vector2<real>{-4.0, 5.5};
    check(output.str() == "1.25 -2.5|-4 5.5",
          "2D point and vector stream output");

    point2<real> input_point;
    vector2<real> input_vector;
    std::istringstream input("1.25 -2.5 -4 5.5");
    input >> input_point >> input_vector;
    check(input_point == point2<real>{1.25, -2.5},
          "2D point stream input");
    check(input_vector == vector2<real>{-4.0, 5.5},
          "2D vector stream input");

    std::ostringstream csv_output;
    const auto original_flags = csv_output.flags();
    const auto original_precision = csv_output.precision();
    point2<real>{1.25, -2.5}.csv_write(
        csv_output, nurbspath::text_format::csv, 3);
    check(csv_output.str() == "1.250e+00,-2.500e+00",
          "2D point CSV output is scientific with requested decimals");
    check(csv_output.flags() == original_flags &&
              csv_output.precision() == original_precision,
          "2D CSV output restores stream formatting");
    std::ostringstream default_csv_output;
    vector2<real>{1.25, -2.5}.csv_write(default_csv_output);
    check(default_csv_output.str() == "1.250000e+00,-2.500000e+00",
          "2D CSV output defaults to six decimals");
    bool rejected_negative_decimals = false;
    try {
        point2<real>{}.csv_write(
            csv_output, nurbspath::text_format::csv, -1);
    } catch (const std::invalid_argument&) {
        rejected_negative_decimals = true;
    }
    check(rejected_negative_decimals,
          "2D CSV output rejects a negative decimal count");
    point2<real> csv_point;
    std::istringstream csv_input("1.25,-2.5");
    csv_point.csv_read(csv_input);
    check(csv_point == point2<real>{1.25, -2.5}, "2D point CSV input");

    std::ostringstream tsv_output;
    vector2<real>{-4.0, 5.5}.csv_write(
        tsv_output, nurbspath::text_format::tsv, 2);
    check(tsv_output.str() == "-4.00e+00\t5.50e+00",
          "2D vector TSV output is scientific with requested decimals");
    vector2<real> tsv_vector;
    std::istringstream tsv_input("-4\t5.5");
    tsv_vector.csv_read(tsv_input, nurbspath::text_format::tsv);
    check(tsv_vector == vector2<real>{-4.0, 5.5}, "2D vector TSV input");

    std::stringstream binary(
        std::ios::in | std::ios::out | std::ios::binary);
    point2<real>{1.25, -2.5}.write(binary);
    vector2<real>{-4.0, 5.5}.write(binary);
    check(binary.str().size() == 4 * sizeof(real),
          "2D binary records contain four scalar values");
    binary.seekg(0);
    point2<real> binary_point;
    vector2<real> binary_vector;
    binary_point.read(binary);
    binary_vector.read(binary);
    check(binary_point == point2<real>{1.25, -2.5},
          "2D point binary round trip");
    check(binary_vector == vector2<real>{-4.0, 5.5},
          "2D vector binary round trip");

    vector2<real> unchanged{7.0, 8.0};
    std::istringstream incomplete_input("1");
    incomplete_input >> unchanged;
    check(incomplete_input.fail() && unchanged == vector2<real>{7.0, 8.0},
          "failed 2D vector input preserves the value");
    std::istringstream malformed_csv("1;2");
    unchanged.csv_read(malformed_csv);
    check(malformed_csv.fail() && unchanged == vector2<real>{7.0, 8.0},
          "malformed 2D CSV input preserves the value");

    const ray2<real> ray(point, {2.0, -1.0});
    static_assert(std::is_same_v<
                  decltype(std::declval<const ray2<real>&>().get_origin()),
                  const point2<real>&>);
    static_assert(std::is_same_v<
                  decltype(std::declval<const ray2<real>&>().get_direction()),
                  const vector2<real>&>);
    static_assert(noexcept(
        std::declval<const ray2<real>&>().get_origin()));
    static_assert(noexcept(
        std::declval<const ray2<real>&>().get_direction()));
    static_assert(noexcept(
        std::declval<ray2<real>&>().set_origin(
            std::declval<const point2<real>&>())));
    static_assert(noexcept(
        std::declval<ray2<real>&>().set_origin(
            std::declval<const vector2<real>&>())));
    const point2<real> ray_origin_before = ray.origin();
    const vector2<real> ray_direction_before = ray.direction();
    check(&ray.get_origin() == &ray.origin() &&
              &ray.get_direction() == &ray.direction(),
          "2D ray get accessors agree with legacy accessors");
    const vector2<real> vector_from_ray{ray};
    check(vector_from_ray == vector2<real>{1.0, 2.0} &&
              ray.origin() == ray_origin_before &&
              ray.direction() == ray_direction_before,
          "2D vector construction copies a ray origin without changing the ray");
    check_point2(ray.evaluate(1.5), {4.0, 0.5}, 1e-12, "2D ray evaluation");
    check(ray.tangent().approximately_equal(
              vector2<real>{2.0, -1.0}.normalized(), 1e-12),
          "2D ray tangent");

    ray2<real> updated_ray({1.0, 2.0}, {3.0, 4.0});
    updated_ray.set_origin({-5.0, 6.0});
    check(updated_ray.get_origin() == point2<real>{-5.0, 6.0} &&
              updated_ray.get_direction() == vector2<real>{3.0, 4.0},
          "2D ray point origin setter preserves direction and accepts braces");
    const vector2<real> replacement_ray_origin{7.0, -8.0};
    updated_ray.set_origin(replacement_ray_origin);
    check(updated_ray.get_origin() == point2<real>{7.0, -8.0} &&
              updated_ray.get_direction() == vector2<real>{3.0, 4.0},
          "2D ray vector origin setter preserves direction");
    updated_ray.set_direction({-2.0, 5.0});
    check(updated_ray.get_origin() == point2<real>{7.0, -8.0} &&
              updated_ray.get_direction() == vector2<real>{-2.0, 5.0},
          "2D ray direction setter preserves origin and accepts braces");

    bool rejected_updated_ray_direction = false;
    try {
        updated_ray.set_direction({1e-4, 0.0}, 1e-3);
    } catch (const std::invalid_argument&) {
        rejected_updated_ray_direction = true;
    }
    check(rejected_updated_ray_direction &&
              updated_ray.get_origin() == point2<real>{7.0, -8.0} &&
              updated_ray.get_direction() == vector2<real>{-2.0, 5.0},
          "2D ray direction setter rejects a near-zero direction and preserves the ray");

    updated_ray.set_origin_and_direction({9.0, 10.0}, {0.0, -3.0});
    check(updated_ray.get_origin() == point2<real>{9.0, 10.0} &&
              updated_ray.get_direction() == vector2<real>{0.0, -3.0},
          "2D ray combined setter accepts a point origin and updates both values");
    const vector2<real> combined_ray_origin{-11.0, 12.0};
    updated_ray.set_origin_and_direction(combined_ray_origin, {4.0, 0.0});
    check(updated_ray.get_origin() == point2<real>{-11.0, 12.0} &&
              updated_ray.get_direction() == vector2<real>{4.0, 0.0},
          "2D ray combined setter accepts a vector origin and updates both values");

    bool rejected_combined_ray_direction = false;
    try {
        updated_ray.set_origin_and_direction({99.0, 100.0}, {0.0, 0.0});
    } catch (const std::invalid_argument&) {
        rejected_combined_ray_direction = true;
    }
    check(rejected_combined_ray_direction &&
              updated_ray.get_origin() == point2<real>{-11.0, 12.0} &&
              updated_ray.get_direction() == vector2<real>{4.0, 0.0},
          "2D ray combined setter validates before changing either value");
    check_point2(updated_ray.evaluate(2.0), {-3.0, 12.0}, 1e-12,
                 "2D ray evaluation uses its updated origin and direction");

    const circle2<real> circle({2.0, 3.0}, 2.0);
    static_assert(std::is_same_v<
                  decltype(std::declval<const circle2<real>&>().get_center()),
                  const point2<real>&>);
    static_assert(std::is_same_v<
                  decltype(std::declval<const circle2<real>&>().get_radius()),
                  real>);
    static_assert(noexcept(
        std::declval<const circle2<real>&>().get_center()));
    static_assert(noexcept(
        std::declval<const circle2<real>&>().get_radius()));
    static_assert(noexcept(
        std::declval<circle2<real>&>().set_center(
            std::declval<const point2<real>&>())));
    static_assert(noexcept(
        std::declval<circle2<real>&>().set_center(
            std::declval<const vector2<real>&>())));
    static_assert(std::is_same_v<
                  decltype(std::declval<circle2<real>&>().set_radius(real{})),
                  void>);
    check(&circle.get_center() == &circle.center() &&
              circle.get_radius() == circle.radius(),
          "circle get accessors agree with legacy accessors");
    const vector2<real> vector_from_circle{circle};
    check(vector_from_circle == vector2<real>{2.0, 3.0} &&
              circle.center() == point2<real>{2.0, 3.0} &&
              circle.radius() == 2.0,
          "2D vector construction copies a circle center without changing it");
    circle2<real> assigned_circle({-1.0, -2.0}, 4.5);
    const vector2<real> circle_assignment_source{6.0, -7.0};
    circle2<real>* assigned_circle_result =
        &(assigned_circle = circle_assignment_source);
    check(assigned_circle_result == &assigned_circle &&
              assigned_circle.center() == point2<real>{6.0, -7.0} &&
              assigned_circle.radius() == 4.5 &&
              circle_assignment_source == vector2<real>{6.0, -7.0},
          "2D circle assignment updates only its center and returns itself");

    circle2<real> updated_circle({1.0, 2.0}, 3.0);
    updated_circle.set_center({4.0, 5.0});
    check(updated_circle.get_center() == point2<real>{4.0, 5.0} &&
              updated_circle.get_radius() == 3.0,
          "circle point center setter preserves radius and accepts braces");
    const vector2<real> updated_center{-6.0, 7.0};
    updated_circle.set_center(updated_center);
    check(updated_circle.get_center() == point2<real>{-6.0, 7.0} &&
              updated_circle.get_radius() == 3.0,
          "circle vector center setter preserves radius");
    updated_circle.set_radius(8.0);
    check(updated_circle.get_center() == point2<real>{-6.0, 7.0} &&
              updated_circle.get_radius() == 8.0,
          "circle radius setter preserves center");
    updated_circle.set_center_and_radius({9.0, -10.0}, 11.0);
    check(updated_circle.get_center() == point2<real>{9.0, -10.0} &&
              updated_circle.get_radius() == 11.0,
          "circle point center-and-radius setter updates both values");
    const vector2<real> replacement_center{-12.0, 13.0};
    updated_circle.set_center_and_radius(replacement_center, 14.0);
    check(updated_circle.get_center() == point2<real>{-12.0, 13.0} &&
              updated_circle.get_radius() == 14.0,
          "circle vector center-and-radius setter updates both values");
    check_point2(
        updated_circle.point_at(0.0),
        {2.0, 13.0},
        1e-12,
        "circle evaluation uses the updated center and radius");

    const real invalid_updated_radii[]{
        0.0,
        -1.0,
        std::numeric_limits<real>::infinity(),
        std::numeric_limits<real>::quiet_NaN()};
    std::size_t rejected_updated_radii = 0;
    for (const real invalid_radius : invalid_updated_radii) {
        try {
            updated_circle.set_radius(invalid_radius);
        } catch (const std::invalid_argument&) {
            ++rejected_updated_radii;
        }
    }
    check(rejected_updated_radii == std::size(invalid_updated_radii) &&
              updated_circle.get_center() == point2<real>{-12.0, 13.0} &&
              updated_circle.get_radius() == 14.0,
          "circle radius setter rejects every invalid radius and preserves the circle");

    bool rejected_combined_radius = false;
    try {
        updated_circle.set_center_and_radius(
            point2<real>{99.0, 100.0},
            std::numeric_limits<real>::quiet_NaN());
    } catch (const std::invalid_argument&) {
        rejected_combined_radius = true;
    }
    check(rejected_combined_radius &&
              updated_circle.get_center() == point2<real>{-12.0, 13.0} &&
              updated_circle.get_radius() == 14.0,
          "circle combined setter validates before changing either value");
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
