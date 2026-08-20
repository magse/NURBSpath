#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

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

    static_assert(std::is_aggregate_v<vector2<real>>);
    vector2<real> components{};
    check(components.x == 0.0 && components.y == 0.0,
          "default 2D vector components are zero");
    components.x = 2.0;
    components.y = -3.0;
    check(components == vector2<real>{2.0, -3.0},
          "2D vector components are public and mutable");

    const vector2<real> first{.x = 3.0, .y = 4.0};
    const vector2<real> second{.x = -2.0, .y = 1.0};
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
