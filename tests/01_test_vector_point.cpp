#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <numbers>
#include <sstream>
#include <string>

int main() {
    using namespace test_support;

    const vector3<real> x{1.0, 0.0, 0.0};
    const vector3<real> y{0.0, 2.0, 0.0};
    check_point(point3<real>{1.0, 2.0, 3.0} + x,
                {2.0, 2.0, 3.0}, 1e-12, "point plus vector");
    check(x.cross(y).approximately_equal({0.0, 0.0, 2.0}, 1e-12),
          "vector cross product");
    check_near(x.dot(y), 0.0, 1e-12, "vector dot product");
    check_near(y.normalized().length(), 1.0, 1e-12, "vector normalization");
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
