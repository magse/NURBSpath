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

    static_assert(std::is_aggregate_v<point3<real>>);
    static_assert(noexcept(point3<real>{}.magnitude()));
    static_assert(noexcept(point3<real>{}.manhattan_distance()));
    static_assert(std::is_same_v<
                  decltype(point3<real>{}.manhattan_distance()), real>);
    point3<real> point_components{};
    check(point_components.x == 0.0 && point_components.y == 0.0 &&
              point_components.z == 0.0,
          "default 3D point coordinates are zero");
    point_components.x = -1.0;
    point_components.y = 2.0;
    point_components.z = 3.0;
    check(point_components == point3<real>{-1.0, 2.0, 3.0},
          "3D point coordinates are public and mutable");
    const point3<real> magnitude_point{.x = 2.0, .y = -3.0, .z = 6.0};
    check_near(magnitude_point.magnitude(), 7.0, 1e-12,
               "3D point magnitude is distance from world origin");
    check_near(magnitude_point.manhattan_distance(), 11.0, 1e-12,
               "3D point Manhattan distance is L1 distance from world origin");
    check(point3<real>::origin().manhattan_distance() == 0.0,
          "3D origin has zero Manhattan distance");

    static_assert(std::is_aggregate_v<vector3<real>>);
    vector3<real> components{};
    check(components.x == 0.0 && components.y == 0.0 && components.z == 0.0,
          "default 3D vector components are zero");
    components.x = -1.0;
    components.y = 2.0;
    components.z = 3.0;
    check(components == vector3<real>{-1.0, 2.0, 3.0},
          "3D vector components are public and mutable");

    const vector3<real> x{.x = 1.0, .y = 0.0, .z = 0.0};
    const vector3<real> y{.x = 0.0, .y = 2.0, .z = 0.0};
    check_point(point3<real>{1.0, 2.0, 3.0} + x,
                {2.0, 2.0, 3.0}, 1e-12, "point plus vector");
    check(x.cross(y).approximately_equal({0.0, 0.0, 2.0}, 1e-12),
          "vector cross product");
    check_near(x.dot(y), 0.0, 1e-12, "vector dot product");
    check_near(y.normalized().length(), 1.0, 1e-12, "vector normalization");
    vector3<real> in_place{0.0, 3.0, 4.0};
    static_assert(std::is_void_v<decltype(in_place.normalize())>);
    in_place.normalize();
    check(in_place.approximately_equal({0.0, 0.6, 0.8}, 1e-12),
          "3D vector in-place normalization preserves direction");
    check_near(in_place.length(), 1.0, 1e-12,
               "3D in-place normalized vector has unit length");
    vector3<real> zero_vector{};
    bool rejected_zero_normalization = false;
    try {
        zero_vector.normalize();
    } catch (const std::domain_error&) {
        rejected_zero_normalization = true;
    }
    check(rejected_zero_normalization && zero_vector == vector3<real>{},
          "3D in-place normalization rejects and preserves zero vector");
    const real large_component = std::numeric_limits<real>::max() / 2.0;
    vector3<real> large_vector{
        large_component, large_component, large_component};
    large_vector.normalize();
    check_near(large_vector.length(), 1.0, 1e-12,
               "3D in-place normalization handles large finite components");
    check_near(x.angle_to(y), std::numbers::pi / 2.0, 1e-12, "vector angle");
    check(x.reflected({1.0, 0.0, 0.0}).approximately_equal(
              {-1.0, 0.0, 0.0}, 1e-12),
          "vector reflection");

    std::ostringstream output;
    output << point3<real>{1.25, -2.5, 3.75} << '|'
           << vector3<real>{-4.0, 5.5, 6.0};
    check(output.str() == "1.25 -2.5 3.75|-4 5.5 6",
          "3D point and vector stream output");

    point3<real> input_point;
    vector3<real> input_vector;
    std::istringstream input("1.25 -2.5 3.75 -4 5.5 6");
    input >> input_point >> input_vector;
    check(input_point == point3<real>{1.25, -2.5, 3.75},
          "3D point stream input");
    check(input_vector == vector3<real>{-4.0, 5.5, 6.0},
          "3D vector stream input");

    std::ostringstream csv_output;
    const auto original_flags = csv_output.flags();
    const auto original_precision = csv_output.precision();
    point3<real>{1.25, -2.5, 3.75}.csv_write(
        csv_output, nurbspath::text_format::csv, 3);
    check(csv_output.str() == "1.250e+00,-2.500e+00,3.750e+00",
          "3D point CSV output is scientific with requested decimals");
    check(csv_output.flags() == original_flags &&
              csv_output.precision() == original_precision,
          "3D CSV output restores stream formatting");
    std::ostringstream default_csv_output;
    vector3<real>{1.25, -2.5, 3.75}.csv_write(default_csv_output);
    check(default_csv_output.str() ==
              "1.250000e+00,-2.500000e+00,3.750000e+00",
          "3D CSV output defaults to six decimals");
    point3<real> csv_point;
    std::istringstream csv_input("1.25,-2.5,3.75");
    csv_point.csv_read(csv_input);
    check(csv_point == point3<real>{1.25, -2.5, 3.75},
          "3D point CSV input");

    std::ostringstream tsv_output;
    vector3<real>{-4.0, 5.5, 6.0}.csv_write(
        tsv_output, nurbspath::text_format::tsv, 2);
    check(tsv_output.str() == "-4.00e+00\t5.50e+00\t6.00e+00",
          "3D vector TSV output is scientific with requested decimals");
    vector3<real> tsv_vector;
    std::istringstream tsv_input("-4\t5.5\t6");
    tsv_vector.csv_read(tsv_input, nurbspath::text_format::tsv);
    check(tsv_vector == vector3<real>{-4.0, 5.5, 6.0},
          "3D vector TSV input");

    std::stringstream binary(
        std::ios::in | std::ios::out | std::ios::binary);
    point3<real>{1.25, -2.5, 3.75}.write(binary);
    vector3<real>{-4.0, 5.5, 6.0}.write(binary);
    check(binary.str().size() == 6 * sizeof(real),
          "3D binary records contain six scalar values");
    binary.seekg(0);
    point3<real> binary_point;
    vector3<real> binary_vector;
    binary_point.read(binary);
    binary_vector.read(binary);
    check(binary_point == point3<real>{1.25, -2.5, 3.75},
          "3D point binary round trip");
    check(binary_vector == vector3<real>{-4.0, 5.5, 6.0},
          "3D vector binary round trip");

    point3<real> unchanged{7.0, 8.0, 9.0};
    std::istringstream incomplete_input("1 2");
    incomplete_input >> unchanged;
    check(incomplete_input.fail() && unchanged == point3<real>{7.0, 8.0, 9.0},
          "failed 3D point input preserves the value");

    std::string incomplete_binary(sizeof(real), '\0');
    std::istringstream incomplete_binary_input(
        incomplete_binary, std::ios::in | std::ios::binary);
    unchanged.read(incomplete_binary_input);
    check(incomplete_binary_input.fail() &&
              unchanged == point3<real>{7.0, 8.0, 9.0},
          "failed 3D binary input preserves the value");
    return finish("01_test_vector_point");
}
