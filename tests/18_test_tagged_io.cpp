#include <nurbspath/config.hpp>
#include <nurbspath/tagged_serialization.hpp>

#include "test_support.hpp"

#include <algorithm>
#include <ios>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

namespace {

using test_support::real;
using test_support::nurbs_spline3;
using test_support::point3;
using test_support::sphere3;
using test_support::vector3;

void check_spline_definition(
    const nurbs_spline3<real>& actual,
    const nurbs_spline3<real>& expected,
    std::string_view message) {
    test_support::check(
        actual.control_points() == expected.control_points() &&
            actual.weights() == expected.weights() &&
            actual.knots() == expected.knots() &&
            actual.degree() == expected.degree() &&
            actual.tolerance() == expected.tolerance() &&
            actual.is_closed() == expected.is_closed(),
        message);
}

void check_malformed(std::string_view row, std::string_view message) {
    std::istringstream input{std::string(row)};
    const auto result = nurbspath::tag_read<real>(input);
    test_support::check(!result && input.fail(), message);
}

} // namespace

int main() {
    using namespace test_support;

    static_assert(std::is_same_v<
                  decltype(point3<real>::tag_read(
                      std::declval<std::istream&>())),
                  std::optional<nurbspath::tagged_read_result<point3<real>>>>);
    static_assert(std::is_same_v<
                  decltype(nurbspath::tag_read<real>(
                      std::declval<std::istream&>())),
                  std::optional<nurbspath::tagged_entity3<real>>>);

    const point3<real> point{1.25, -2.5, 3.75};
    const vector3<real> vector{-4.0, 5.5, 6.0};
    const sphere3<real> sphere({1.0, -2.0, 3.5}, 4.25);
    const nurbs_spline3<real> spline(
        {{0.0, 0.0, 0.0}, {1.0, -2.0, 3.0}, {4.0, 5.0, -6.0}},
        {1.0, 0.5, 2.0},
        {-1.0, -1.0, -1.0, 2.0, 2.0, 2.0},
        2,
        false,
        0.125);

    std::ostringstream point_output;
    point_output.setf(std::ios::hex, std::ios::basefield);
    point_output.setf(std::ios::fixed, std::ios::floatfield);
    point_output.setf(std::ios::showpos | std::ios::uppercase);
    point_output.precision(3);
    const auto point_flags = point_output.flags();
    const auto point_precision = point_output.precision();
    std::ostream& point_stream = point.tag_write(42, point_output);
    check(&point_stream == &point_output,
          "point tag_write returns the destination stream");
    check(
        point_output.str() ==
            "42 point3 v1 1.2500000000000000e+00 "
            "-2.5000000000000000e+00 3.7500000000000000e+00\n",
        "point tagged row is canonical scientific text");
    check(point_output.flags() == point_flags &&
              point_output.precision() == point_precision,
          "point tag_write preserves caller stream formatting");

    std::ostringstream vector_output;
    vector.tag_write(42, vector_output);
    check(
        vector_output.str() ==
            "42 vector3 v1 -4.0000000000000000e+00 "
            "5.5000000000000000e+00 6.0000000000000000e+00\n",
        "vector tagged row is canonical scientific text");

    std::ostringstream sphere_output;
    sphere.tag_write(42, sphere_output);
    check(
        sphere_output.str() ==
            "42 sphere3 v1 1.0000000000000000e+00 "
            "-2.0000000000000000e+00 3.5000000000000000e+00 "
            "4.2500000000000000e+00\n",
        "sphere tagged row is canonical scientific text");

    std::ostringstream spline_output;
    spline.tag_write(42, spline_output);
    const std::string spline_row = spline_output.str();
    check(
        spline_row ==
            "42 spline3 v1 2 0 1.2500000000000000e-01 3 6 "
            "0.0000000000000000e+00 0.0000000000000000e+00 "
            "0.0000000000000000e+00 1.0000000000000000e+00 "
            "1.0000000000000000e+00 -2.0000000000000000e+00 "
            "3.0000000000000000e+00 5.0000000000000000e-01 "
            "4.0000000000000000e+00 5.0000000000000000e+00 "
            "-6.0000000000000000e+00 2.0000000000000000e+00 "
            "-1.0000000000000000e+00 -1.0000000000000000e+00 "
            "-1.0000000000000000e+00 2.0000000000000000e+00 "
            "2.0000000000000000e+00 2.0000000000000000e+00\n",
        "spline tagged row contains its complete definition on one row");
    check(std::count(spline_row.begin(), spline_row.end(), '\n') == 1,
          "spline tag_write emits exactly one physical row");

    std::istringstream typed_point_input(point_output.str());
    const auto typed_point = point3<real>::tag_read(typed_point_input);
    check(typed_point && typed_point->tag == 42 && typed_point->entity &&
              *typed_point->entity == point,
          "typed point reader round trips tag and coordinates");

    const real subnormal = std::numeric_limits<real>::denorm_min();
    if (subnormal > 0.0) {
        const point3<real> subnormal_point{
            subnormal, -subnormal, std::numeric_limits<real>::min()};
        std::stringstream subnormal_stream;
        subnormal_point.tag_write(43, subnormal_stream);
        const auto decoded_subnormal = point3<real>::tag_read(subnormal_stream);
        check(decoded_subnormal && decoded_subnormal->entity &&
                  *decoded_subnormal->entity == subnormal_point,
              "tagged REAL parsing exactly round trips subnormal values");
    }

    const nurbspath::point3<float> float_point{1.25F, -2.5F, 3.75F};
    std::stringstream float_stream;
    float_point.tag_write(44, float_stream);
    const auto decoded_float = nurbspath::point3<float>::tag_read(float_stream);
    check(decoded_float && decoded_float->entity &&
              *decoded_float->entity == float_point,
          "tagged point rows round trip float definitions");

    const long double long_double_subnormal =
        std::numeric_limits<long double>::denorm_min();
    const nurbspath::point3<long double> long_double_point{
        long_double_subnormal > 0.0L ? long_double_subnormal : 1.25L,
        -2.5L,
        3.75L};
    std::stringstream long_double_stream;
    long_double_point.tag_write(45, long_double_stream);
    const auto decoded_long_double =
        nurbspath::point3<long double>::tag_read(long_double_stream);
    check(decoded_long_double && decoded_long_double->entity &&
              *decoded_long_double->entity == long_double_point,
          "tagged point rows round trip long double definitions and subnormals");

    std::istringstream typed_vector_input(vector_output.str());
    const auto typed_vector = vector3<real>::tag_read(typed_vector_input);
    check(typed_vector && typed_vector->tag == 42 && typed_vector->entity &&
              *typed_vector->entity == vector,
          "typed vector reader round trips tag and components");

    std::istringstream typed_sphere_input(sphere_output.str());
    const auto typed_sphere = sphere3<real>::tag_read(typed_sphere_input);
    check(typed_sphere && typed_sphere->tag == 42 && typed_sphere->entity &&
              typed_sphere->entity->center() == sphere.center() &&
              typed_sphere->entity->radius() == sphere.radius(),
          "typed sphere reader round trips tag and definition");

    std::istringstream typed_spline_input(spline_output.str());
    const auto typed_spline = nurbs_spline3<real>::tag_read(typed_spline_input);
    check(typed_spline && typed_spline->tag == 42 && typed_spline->entity,
          "typed spline reader allocates the tagged spline");
    if (typed_spline && typed_spline->entity) {
        check_spline_definition(
            *typed_spline->entity, spline,
            "typed spline reader preserves its complete definition");
    }

    const nurbs_spline3<real> closed_spline(
        {{0.0, 0.0, 0.0}, {1.0, 2.0, 0.0}, {0.0, 0.0, 0.0}},
        {1.0, 1.5, 1.0},
        {0.0, 0.0, 1.0, 2.0, 2.0},
        1,
        true,
        1e-12);
    std::stringstream closed_spline_stream;
    closed_spline.tag_write(9, closed_spline_stream);
    const auto decoded_closed = nurbs_spline3<real>::tag_read(
        closed_spline_stream);
    check(decoded_closed && decoded_closed->entity &&
              decoded_closed->entity->is_closed(),
          "typed spline reader preserves closure");
    if (decoded_closed && decoded_closed->entity) {
        check_spline_definition(
            *decoded_closed->entity, closed_spline,
            "closed spline round trip preserves every stored field");
    }

    std::stringstream repeated_tag_rows;
    point.tag_write(77, repeated_tag_rows);
    vector.tag_write(77, repeated_tag_rows);
    sphere.tag_write(77, repeated_tag_rows);
    spline.tag_write(77, repeated_tag_rows);
    const std::string repeated_rows_text = repeated_tag_rows.str();
    check(std::count(
              repeated_rows_text.begin(), repeated_rows_text.end(), '\n') == 4,
          "repeated tag writes append four independent rows");

    const auto general_point = nurbspath::tag_read<real>(repeated_tag_rows);
    const auto general_vector = nurbspath::tag_read<real>(repeated_tag_rows);
    const auto general_sphere = nurbspath::tag_read<real>(repeated_tag_rows);
    const auto general_spline = nurbspath::tag_read<real>(repeated_tag_rows);
    check(general_point && general_point->tag == 77 &&
              std::holds_alternative<std::shared_ptr<point3<real>>>(
                  general_point->entity) &&
              *std::get<std::shared_ptr<point3<real>>>(general_point->entity) ==
                  point,
          "general reader allocates a point and preserves a repeated tag");
    check(general_vector && general_vector->tag == 77 &&
              std::holds_alternative<std::shared_ptr<vector3<real>>>(
                  general_vector->entity) &&
              *std::get<std::shared_ptr<vector3<real>>>(
                  general_vector->entity) == vector,
          "general reader allocates a vector and preserves a repeated tag");
    check(general_sphere && general_sphere->tag == 77 &&
              std::holds_alternative<std::shared_ptr<sphere3<real>>>(
                  general_sphere->entity),
          "general reader allocates a sphere and preserves a repeated tag");
    if (general_sphere &&
        std::holds_alternative<std::shared_ptr<sphere3<real>>>(
            general_sphere->entity)) {
        const auto& decoded =
            *std::get<std::shared_ptr<sphere3<real>>>(general_sphere->entity);
        check(decoded.center() == sphere.center() &&
                  decoded.radius() == sphere.radius(),
              "general sphere definition round trips");
    }
    check(general_spline && general_spline->tag == 77 &&
              std::holds_alternative<std::shared_ptr<nurbs_spline3<real>>>(
                  general_spline->entity),
          "general reader allocates a spline and preserves a repeated tag");
    if (general_spline &&
        std::holds_alternative<std::shared_ptr<nurbs_spline3<real>>>(
            general_spline->entity)) {
        check_spline_definition(
            *std::get<std::shared_ptr<nurbs_spline3<real>>>(
                general_spline->entity),
            spline,
            "general spline definition round trips");
    }

    const auto at_eof = nurbspath::tag_read<real>(repeated_tag_rows);
    check(!at_eof && repeated_tag_rows.eof() && !repeated_tag_rows.bad(),
          "general reader returns nullopt at clean EOF");

    std::istringstream crlf_input(
        "88 point3 v1 1.25e+00 -2.5e+00 3.75e+00\r\n");
    const auto crlf_point = point3<real>::tag_read(crlf_input);
    check(crlf_point && crlf_point->tag == 88 && crlf_point->entity &&
              *crlf_point->entity == point,
          "typed reader accepts CRLF line termination");

    std::istringstream final_row_without_newline(
        "89 vector3 v1 -4 5.5 6");
    const auto final_vector = vector3<real>::tag_read(
        final_row_without_newline);
    check(final_vector && final_vector->tag == 89 && final_vector->entity &&
              *final_vector->entity == vector &&
              final_row_without_newline.eof() &&
              !final_row_without_newline.fail(),
          "typed reader accepts a final row without a line ending");

    check_malformed("\n", "blank tagged row is rejected");
    check_malformed("-1 point3 v1 0 0 0\n", "negative tag is rejected");
    std::ostringstream overflowing_tag;
    overflowing_tag << std::numeric_limits<std::size_t>::max() << '0';
    check_malformed(
        overflowing_tag.str() + " point3 v1 0 0 0\n",
        "overflowing tag is rejected");
    check_malformed(
        "1 Point3 v1 0 0 0\n", "entity type is case-sensitive");
    check_malformed(
        "1 ray3 v1 0 0 0\n", "unknown entity type is rejected");
    check_malformed(
        "1 point3 v2 0 0 0\n", "unknown format version is rejected");
    check_malformed(
        "1 point3 v1 0 0\n", "truncated point payload is rejected");
    check_malformed(
        "1 point3 v1 0 0 0 extra\n", "trailing fields are rejected");
    check_malformed(
        "1 point3 v1 nan 0 0\n", "non-finite point input is rejected");
    check_malformed(
        "1 sphere3 v1 0 0 0 -1\n", "nonpositive sphere radius is rejected");
    check_malformed(
        "1 spline3 v1 1 2 1e-12 2 4 "
        "0 0 0 1 1 0 0 1 0 0 1 1\n",
        "invalid spline closure flag is rejected");
    check_malformed(
        "1 spline3 v1 1 0 1e-12 2 4 "
        "0 0 0 1 1 0 0 -1 0 0 1 1\n",
        "invalid spline weight is rejected");
    check_malformed(
        "1 spline3 v1 1 0 1e-12 999999999999999999 "
        "1000000000000000001\n",
        "unreasonable spline counts fail before allocation");
    check_malformed(
        "1 spline3 v1 1 0 1e-12 2 4 0\n",
        "declared spline counts require matching payload fields");

    std::istringstream wrong_typed_input(
        "3 vector3 v1 1 2 3\n");
    const auto wrong_typed = point3<real>::tag_read(wrong_typed_input);
    check(!wrong_typed && wrong_typed_input.fail(),
          "typed reader rejects a different entity type");

    std::istringstream isolated_rows(
        "1 point3 v1 1 2\n"
        "2 point3 v1 3 4 5\n");
    const auto malformed_first = nurbspath::tag_read<real>(isolated_rows);
    check(!malformed_first && isolated_rows.fail(),
          "malformed row reports failbit");
    isolated_rows.clear();
    const auto valid_second = nurbspath::tag_read<real>(isolated_rows);
    check(valid_second && valid_second->tag == 2 &&
              std::holds_alternative<std::shared_ptr<point3<real>>>(
                  valid_second->entity) &&
              *std::get<std::shared_ptr<point3<real>>>(valid_second->entity) ==
                  point3<real>{3.0, 4.0, 5.0},
          "malformed row does not consume the following physical row");

    bool rejected_nonfinite_point = false;
    try {
        point3<real>{std::numeric_limits<real>::infinity(), 0.0, 0.0}
            .tag_write(1, point_output);
    } catch (const std::invalid_argument&) {
        rejected_nonfinite_point = true;
    }
    check(rejected_nonfinite_point,
          "point writer rejects non-finite coordinates");

    bool rejected_nonfinite_vector = false;
    try {
        vector3<real>{0.0, std::numeric_limits<real>::quiet_NaN(), 0.0}
            .tag_write(1, vector_output);
    } catch (const std::invalid_argument&) {
        rejected_nonfinite_vector = true;
    }
    check(rejected_nonfinite_vector,
          "vector writer rejects non-finite components");

    bool rejected_nonfinite_sphere = false;
    try {
        sphere3<real>{{0.0, 0.0, 0.0},
                      std::numeric_limits<real>::infinity()}
            .tag_write(1, sphere_output);
    } catch (const std::invalid_argument&) {
        rejected_nonfinite_sphere = true;
    }
    check(rejected_nonfinite_sphere,
          "sphere writer rejects non-finite radius");

    return finish("18_test_tagged_io");
}
