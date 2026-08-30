#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <array>
#include <concepts>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using real = test_support::real;
using spline2 = nurbspath::nurbs_spline2<real>;
using spline3 = nurbspath::nurbs_spline3<real>;

static_assert(std::same_as<
              decltype(std::declval<const spline2&>().number_of_parameters()),
              std::size_t>);
static_assert(std::same_as<
              decltype(std::declval<const spline3&>().number_of_parameters()),
              std::size_t>);
static_assert(noexcept(
    std::declval<const spline2&>().number_of_parameters()));
static_assert(noexcept(
    std::declval<const spline3&>().number_of_parameters()));
static_assert(std::same_as<
              decltype(std::declval<const spline2&>().get_parameter(0)),
              real>);
static_assert(std::same_as<
              decltype(std::declval<const spline3&>().get_parameter(0)),
              real>);
static_assert(std::same_as<
              decltype(std::declval<const spline2&>().parameter_name(0)),
              std::string>);
static_assert(std::same_as<
              decltype(std::declval<const spline3&>().parameter_name(0)),
              std::string>);
static_assert(std::same_as<
              decltype(std::declval<spline2&>().set_parameter(0, real{})),
              bool>);
static_assert(std::same_as<
              decltype(std::declval<spline3&>().set_parameter(0, real{})),
              bool>);

template <typename EXCEPTION, typename FUNCTION>
bool throws_exception(FUNCTION&& function) {
    try {
        std::forward<FUNCTION>(function)();
    } catch (const EXCEPTION&) {
        return true;
    } catch (...) {
    }
    return false;
}

template <typename SPLINE>
struct spline_snapshot {
    using definition_type = std::remove_cvref_t<decltype(
        std::declval<const SPLINE&>().definition())>;
    using point_type = typename std::remove_cvref_t<decltype(
        std::declval<definition_type>().control_points)>::value_type;

    explicit spline_snapshot(const SPLINE& spline)
        : definition(spline.definition()),
          degree(spline.degree()),
          start(spline.get_start()),
          end(spline.get_end()),
          sample_s(spline.s_min() +
                   (spline.s_max() - spline.s_min()) * real(0.375)),
          sample(spline.evaluate(sample_s)) {}

    [[nodiscard]] bool matches(const SPLINE& spline) const {
        const definition_type current = spline.definition();
        return current.control_points == definition.control_points &&
               current.weights == definition.weights &&
               current.knots == definition.knots &&
               current.closed == definition.closed &&
               current.tolerance == definition.tolerance &&
               spline.degree() == degree && spline.get_start() == start &&
               spline.get_end() == end && spline.evaluate(sample_s) == sample;
    }

    definition_type definition;
    std::size_t degree;
    point_type start;
    point_type end;
    real sample_s;
    point_type sample;
};

template <typename SPLINE>
void expect_rejected(
    SPLINE& spline,
    std::size_t index,
    real value,
    std::string_view message) {
    const spline_snapshot<SPLINE> before(spline);
    bool accepted = false;
    bool threw = false;
    try {
        accepted = spline.set_parameter(index, value);
    } catch (...) {
        threw = true;
    }
    test_support::check(!threw && !accepted && before.matches(spline), message);
}

[[nodiscard]] spline2 make_open2() {
    return {
        {{10.0, 11.0}, {20.0, 21.0}, {30.0, 31.0}, {40.0, 41.0}},
        {1.0, 2.0, 3.0, 4.0},
        {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0},
        2,
        false,
        1e-9};
}

[[nodiscard]] spline3 make_open3() {
    return {
        {{10.0, 11.0, 12.0},
         {20.0, 21.0, 22.0},
         {30.0, 31.0, 32.0},
         {40.0, 41.0, 42.0}},
        {1.0, 2.0, 3.0, 4.0},
        {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0},
        2,
        false,
        1e-9};
}

void test_mapping2() {
    using namespace test_support;

    spline2 spline = make_open2();
    constexpr std::array<std::string_view, 19> names{
        "P[0].x", "P[0].y", "P[1].x", "P[1].y",
        "P[2].x", "P[2].y", "P[3].x", "P[3].y",
        "W[0]", "W[1]", "W[2]", "W[3]",
        "K[0]", "K[1]", "K[2]", "K[3]", "K[4]", "K[5]", "K[6]"};
    constexpr std::array<real, 19> values{
        10.0, 11.0, 20.0, 21.0, 30.0, 31.0, 40.0, 41.0,
        1.0, 2.0, 3.0, 4.0,
        0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0};

    check(spline.number_of_parameters() == names.size(),
          "2D scalar parameter count includes coordinates, weights, and knots");
    const spline_snapshot<spline2> before_same_value_updates(spline);
    bool all_names_and_values_match = true;
    bool all_same_value_updates_succeed = true;
    for (std::size_t index = 0; index < names.size(); ++index) {
        all_names_and_values_match =
            all_names_and_values_match &&
            spline.parameter_name(index) == names[index] &&
            spline.get_parameter(index) == values[index];
        all_same_value_updates_succeed =
            all_same_value_updates_succeed &&
            spline.set_parameter(index, values[index]);
    }
    check(all_names_and_values_match,
          "2D scalar parameters have the exact documented names and order");
    check(all_same_value_updates_succeed &&
              before_same_value_updates.matches(spline),
          "2D every mapped scalar accepts its unchanged value");
    check(spline.degree() == 2 && !spline.is_closed(),
          "2D indexed parameters exclude degree, closure, and tolerance");

    check(throws_exception<std::out_of_range>([&spline] {
              static_cast<void>(
                  spline.get_parameter(spline.number_of_parameters()));
          }),
          "2D getter rejects the one-past-end parameter index");
    check(throws_exception<std::out_of_range>([&spline] {
              static_cast<void>(spline.parameter_name(
                  std::numeric_limits<std::size_t>::max()));
          }),
          "2D name getter rejects a very large parameter index");
    expect_rejected(
        spline,
        spline.number_of_parameters(),
        1.0,
        "2D setter rejects one-past-end without changing the spline");
    expect_rejected(
        spline,
        std::numeric_limits<std::size_t>::max(),
        1.0,
        "2D setter rejects a very large index without changing the spline");
}

void test_mapping3() {
    using namespace test_support;

    spline3 spline = make_open3();
    constexpr std::array<std::string_view, 23> names{
        "P[0].x", "P[0].y", "P[0].z",
        "P[1].x", "P[1].y", "P[1].z",
        "P[2].x", "P[2].y", "P[2].z",
        "P[3].x", "P[3].y", "P[3].z",
        "W[0]", "W[1]", "W[2]", "W[3]",
        "K[0]", "K[1]", "K[2]", "K[3]", "K[4]", "K[5]", "K[6]"};
    constexpr std::array<real, 23> values{
        10.0, 11.0, 12.0, 20.0, 21.0, 22.0,
        30.0, 31.0, 32.0, 40.0, 41.0, 42.0,
        1.0, 2.0, 3.0, 4.0,
        0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0};

    check(spline.number_of_parameters() == names.size(),
          "3D scalar parameter count includes coordinates, weights, and knots");
    const spline_snapshot<spline3> before_same_value_updates(spline);
    bool all_names_and_values_match = true;
    bool all_same_value_updates_succeed = true;
    for (std::size_t index = 0; index < names.size(); ++index) {
        all_names_and_values_match =
            all_names_and_values_match &&
            spline.parameter_name(index) == names[index] &&
            spline.get_parameter(index) == values[index];
        all_same_value_updates_succeed =
            all_same_value_updates_succeed &&
            spline.set_parameter(index, values[index]);
    }
    check(all_names_and_values_match,
          "3D scalar parameters have the exact documented names and order");
    check(all_same_value_updates_succeed &&
              before_same_value_updates.matches(spline),
          "3D every mapped scalar accepts its unchanged value");
    check(spline.degree() == 2 && !spline.is_closed(),
          "3D indexed parameters exclude degree, closure, and tolerance");

    check(throws_exception<std::out_of_range>([&spline] {
              static_cast<void>(
                  spline.get_parameter(spline.number_of_parameters()));
          }),
          "3D getter rejects the one-past-end parameter index");
    check(throws_exception<std::out_of_range>([&spline] {
              static_cast<void>(spline.parameter_name(
                  std::numeric_limits<std::size_t>::max()));
          }),
          "3D name getter rejects a very large parameter index");
    expect_rejected(
        spline,
        spline.number_of_parameters(),
        1.0,
        "3D setter rejects one-past-end without changing the spline");
    expect_rejected(
        spline,
        std::numeric_limits<std::size_t>::max(),
        1.0,
        "3D setter rejects a very large index without changing the spline");
}

void test_successful_updates2() {
    using namespace test_support;

    spline2 spline = make_open2();
    const point2<real> before_shape = spline.evaluate(0.75);
    check(spline.set_parameter(3, 25.0) &&
              spline.control_point(1).y == 25.0 &&
              spline.evaluate(0.75) != before_shape,
          "2D coordinate parameter changes the evaluated path");
    check(spline.set_parameter(0, 9.0) &&
              spline.get_start() == point2<real>{9.0, 11.0},
          "2D first control coordinate refreshes the cached start");
    check(spline.set_parameter(7, 49.0) &&
              spline.get_end() == point2<real>{40.0, 49.0},
          "2D last control coordinate refreshes the cached end");
    check(spline.set_parameter(10, 5.0) && spline.weight(2) == 5.0,
          "2D weight parameter commits a valid positive weight");
    check(spline.set_parameter(15, 1.25) && spline.knot(3) == 1.25,
          "2D knot parameter commits a valid ordered knot");
    check(spline.degree() == 2 && !spline.is_closed() &&
              spline.tolerance() == 1e-9 &&
              spline.number_of_parameters() == 19,
          "2D successful scalar edits retain non-parameterized state");
}

void test_successful_updates3() {
    using namespace test_support;

    spline3 spline = make_open3();
    const point3<real> before_shape = spline.evaluate(0.75);
    check(spline.set_parameter(5, 25.0) &&
              spline.control_point(1).z == 25.0 &&
              spline.evaluate(0.75) != before_shape,
          "3D coordinate parameter changes the evaluated path");
    check(spline.set_parameter(2, 9.0) &&
              spline.get_start() == point3<real>{10.0, 11.0, 9.0},
          "3D first control coordinate refreshes the cached start");
    check(spline.set_parameter(9, 49.0) &&
              spline.get_end() == point3<real>{49.0, 41.0, 42.0},
          "3D last control coordinate refreshes the cached end");
    check(spline.set_parameter(14, 5.0) && spline.weight(2) == 5.0,
          "3D weight parameter commits a valid positive weight");
    check(spline.set_parameter(19, 1.25) && spline.knot(3) == 1.25,
          "3D knot parameter commits a valid ordered knot");
    check(spline.degree() == 2 && !spline.is_closed() &&
              spline.tolerance() == 1e-9 &&
              spline.number_of_parameters() == 23,
          "3D successful scalar edits retain non-parameterized state");
}

void test_rejected_updates2() {
    spline2 spline = make_open2();
    const real nan = std::numeric_limits<real>::quiet_NaN();
    const real infinity = std::numeric_limits<real>::infinity();

    expect_rejected(spline, 0, nan, "2D rejects a NaN coordinate atomically");
    expect_rejected(spline, 1, infinity, "2D rejects an infinite coordinate atomically");
    expect_rejected(spline, 8, 0.0, "2D rejects a zero weight atomically");
    expect_rejected(spline, 8, -1.0, "2D rejects a negative weight atomically");
    expect_rejected(spline, 8, nan, "2D rejects a NaN weight atomically");
    expect_rejected(spline, 8, infinity, "2D rejects an infinite weight atomically");
    expect_rejected(
        spline,
        8,
        1e-30,
        "2D converts a homogeneous endpoint failure into false with rollback");
    expect_rejected(spline, 15, -0.5, "2D rejects an out-of-order knot atomically");
    expect_rejected(spline, 15, nan, "2D rejects a NaN knot atomically");
    expect_rejected(spline, 15, infinity, "2D rejects an infinite knot atomically");
    spline2 zero_width_candidate{
        {{0.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {3.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0},
        2};
    expect_rejected(
        zero_width_candidate,
        16,
        0.0,
        "2D rejects a knot update that collapses the active domain");
}

void test_rejected_updates3() {
    spline3 spline = make_open3();
    const real nan = std::numeric_limits<real>::quiet_NaN();
    const real infinity = std::numeric_limits<real>::infinity();

    expect_rejected(spline, 0, nan, "3D rejects a NaN coordinate atomically");
    expect_rejected(spline, 2, infinity, "3D rejects an infinite coordinate atomically");
    expect_rejected(spline, 12, 0.0, "3D rejects a zero weight atomically");
    expect_rejected(spline, 12, -1.0, "3D rejects a negative weight atomically");
    expect_rejected(spline, 12, nan, "3D rejects a NaN weight atomically");
    expect_rejected(spline, 12, infinity, "3D rejects an infinite weight atomically");
    expect_rejected(
        spline,
        12,
        1e-30,
        "3D converts a homogeneous endpoint failure into false with rollback");
    expect_rejected(spline, 19, -0.5, "3D rejects an out-of-order knot atomically");
    expect_rejected(spline, 19, nan, "3D rejects a NaN knot atomically");
    expect_rejected(spline, 19, infinity, "3D rejects an infinite knot atomically");
    spline3 zero_width_candidate{
        {{0.0, 0.0, 0.0},
         {1.0, 1.0, 0.0},
         {2.0, 1.0, 0.0},
         {3.0, 0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0},
        2};
    expect_rejected(
        zero_width_candidate,
        20,
        0.0,
        "3D rejects a knot update that collapses the active domain");
}

void test_closed_behavior2() {
    using namespace test_support;

    spline2 spline{
        {{0.0, 0.0}, {1.0, 2.0}, {2.0, 1.0}, {1e-8, 0.0}},
        {1.0, 2.0, 3.0, 1.0},
        {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0},
        2,
        true,
        1e-9};

    expect_rejected(
        spline, 0, 0.25,
        "2D closed spline rejects an isolated first endpoint edit");
    expect_rejected(
        spline, 6, 0.25,
        "2D closed spline rejects an isolated final endpoint edit");
    check(spline.set_parameter(3, 2.5) && spline.control_point(1).y == 2.5,
          "2D closed spline accepts a valid interior coordinate edit");
    check(spline.set_parameter(9, 2.5) && spline.weight(1) == 2.5,
          "2D closed spline accepts a valid weight edit");
    check(spline.is_closed() && spline.degree() == 2 &&
              spline.tolerance() == 1e-9 &&
              spline.number_of_parameters() == 19,
          "2D closed state remains outside the scalar parameter list");
}

void test_closed_behavior3() {
    using namespace test_support;

    spline3 spline{
        {{0.0, 0.0, 0.0},
         {1.0, 2.0, 3.0},
         {2.0, 1.0, 2.0},
         {1e-8, 0.0, 0.0}},
        {1.0, 2.0, 3.0, 1.0},
        {0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0},
        2,
        true,
        1e-9};

    expect_rejected(
        spline, 2, 0.25,
        "3D closed spline rejects an isolated first endpoint edit");
    expect_rejected(
        spline, 9, 0.25,
        "3D closed spline rejects an isolated final endpoint edit");
    check(spline.set_parameter(5, 3.5) && spline.control_point(1).z == 3.5,
          "3D closed spline accepts a valid interior coordinate edit");
    check(spline.set_parameter(13, 2.5) && spline.weight(1) == 2.5,
          "3D closed spline accepts a valid weight edit");
    check(spline.is_closed() && spline.degree() == 2 &&
              spline.tolerance() == 1e-9 &&
              spline.number_of_parameters() == 23,
          "3D closed state remains outside the scalar parameter list");
}

void test_dynamic_counts() {
    using namespace test_support;

    spline2 spline_2d = make_open2();
    spline_2d.set_definition({
        {{0.0, 0.0},
         {1.0, 2.0},
         {2.0, 3.0},
         {3.0, 2.0},
         {4.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0},
        false,
        1e-8});
    check(spline_2d.number_of_parameters() == 23 &&
              spline_2d.parameter_name(9) == "P[4].y" &&
              spline_2d.parameter_name(10) == "W[0]" &&
              spline_2d.parameter_name(14) == "W[4]" &&
              spline_2d.parameter_name(15) == "K[0]" &&
              spline_2d.parameter_name(22) == "K[7]",
          "2D parameter count and group boundaries follow a replaced definition");

    spline3 spline_3d = make_open3();
    spline_3d.set_definition({
        {{0.0, 0.0, 0.0},
         {1.0, 2.0, 1.0},
         {2.0, 3.0, 2.0},
         {3.0, 2.0, 1.0},
         {4.0, 0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0},
        false,
        1e-8});
    check(spline_3d.number_of_parameters() == 28 &&
              spline_3d.parameter_name(14) == "P[4].z" &&
              spline_3d.parameter_name(15) == "W[0]" &&
              spline_3d.parameter_name(19) == "W[4]" &&
              spline_3d.parameter_name(20) == "K[0]" &&
              spline_3d.parameter_name(27) == "K[7]",
          "3D parameter count and group boundaries follow a replaced definition");
}

} // namespace

int main() {
    test_mapping2();
    test_mapping3();
    test_successful_updates2();
    test_successful_updates3();
    test_rejected_updates2();
    test_rejected_updates3();
    test_closed_behavior2();
    test_closed_behavior3();
    test_dynamic_counts();
    return test_support::finish("spline scalar parameter tests");
}
