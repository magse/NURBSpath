#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <iterator>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <valarray>
#include <vector>

namespace {

using real = test_support::real;
using spline2 = nurbspath::nurbs_spline2<real>;
using spline3 = nurbspath::nurbs_spline3<real>;
using arr_spline2 = nurbspath::nurbs_arr_spline2<real>;
using arr_spline3 = nurbspath::nurbs_arr_spline3<real>;

template <typename CANVAS, typename ENTITY>
concept directly_addable = requires(CANVAS& canvas, const ENTITY& entity) {
    canvas.add(entity);
};

template <typename SPLINE>
concept has_named_collection_getters = requires(const SPLINE& spline) {
    spline.get_control_points();
    spline.get_weights();
    spline.get_knots();
};

template <typename SPLINE>
concept has_legacy_control_points_getter = requires(SPLINE& spline) {
    spline.control_points();
};

template <typename SPLINE>
concept has_legacy_weights_getter = requires(SPLINE& spline) {
    spline.weights();
};

template <typename SPLINE>
concept has_legacy_knots_getter = requires(SPLINE& spline) {
    spline.knots();
};

template <typename SPLINE>
concept has_legacy_control_point_getter = requires(SPLINE& spline) {
    spline.control_point(std::size_t{});
};

template <typename SPLINE>
concept has_legacy_weight_getter = requires(SPLINE& spline) {
    spline.weight(std::size_t{});
};

template <typename SPLINE>
concept has_legacy_knot_getter = requires(SPLINE& spline) {
    spline.knot(std::size_t{});
};

static_assert(std::same_as<typename arr_spline2::real_t, real>);
static_assert(std::same_as<typename arr_spline3::real_t, real>);
static_assert(std::same_as<
              typename arr_spline2::parameter_index_range,
              std::ranges::iota_view<std::size_t, std::size_t>>);
static_assert(std::same_as<
              typename arr_spline3::parameter_index_range,
              std::ranges::iota_view<std::size_t, std::size_t>>);
static_assert(std::ranges::random_access_range<
              typename arr_spline2::parameter_index_range>);
static_assert(std::ranges::sized_range<
              typename arr_spline3::parameter_index_range>);
static_assert(std::same_as<
              std::ranges::range_value_t<
                  typename arr_spline2::parameter_index_range>,
              std::size_t>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline2&>()
                           .get_control_point_count_range()),
              typename arr_spline2::parameter_index_range>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline2&>()
                           .get_weight_count_range()),
              typename arr_spline2::parameter_index_range>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline2&>()
                           .get_knot_count_range()),
              typename arr_spline2::parameter_index_range>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline3&>()
                           .get_control_point_count_range()),
              typename arr_spline3::parameter_index_range>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline3&>()
                           .get_weight_count_range()),
              typename arr_spline3::parameter_index_range>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline3&>()
                           .get_knot_count_range()),
              typename arr_spline3::parameter_index_range>);
static_assert(std::same_as<
              decltype(std::declval<arr_spline2&>().parameters),
              std::valarray<real>>);
static_assert(std::same_as<
              decltype(std::declval<arr_spline3&>().parameters),
              std::valarray<real>>);
static_assert(std::same_as<
              decltype(static_cast<void (arr_spline2::*)(const real*)>(
                  &arr_spline2::set_parameters)),
              void (arr_spline2::*)(const real*)>);
static_assert(std::same_as<
              decltype(static_cast<void (arr_spline3::*)(const real*)>(
                  &arr_spline3::set_parameters)),
              void (arr_spline3::*)(const real*)>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline2&>().curvature(real{})),
              real>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline3&>().curvature(real{})),
              real>);
static_assert(!std::same_as<arr_spline2, spline2>);
static_assert(!std::same_as<arr_spline3, spline3>);
static_assert(!std::derived_from<arr_spline2, spline2>);
static_assert(!std::derived_from<spline2, arr_spline2>);
static_assert(!std::derived_from<arr_spline3, spline3>);
static_assert(!std::derived_from<spline3, arr_spline3>);
static_assert(std::copy_constructible<arr_spline2>);
static_assert(std::copy_constructible<arr_spline3>);
static_assert(std::assignable_from<arr_spline2&, const arr_spline2&>);
static_assert(std::assignable_from<arr_spline3&, const arr_spline3&>);
static_assert(std::constructible_from<arr_spline2, const spline2&>);
static_assert(std::constructible_from<arr_spline3, const spline3&>);
static_assert(std::constructible_from<spline2, const arr_spline2&>);
static_assert(std::constructible_from<spline3, const arr_spline3&>);
static_assert(!std::convertible_to<spline2, arr_spline2>);
static_assert(!std::convertible_to<spline3, arr_spline3>);
static_assert(!std::convertible_to<arr_spline2, spline2>);
static_assert(!std::convertible_to<arr_spline3, spline3>);
static_assert(has_named_collection_getters<spline2>);
static_assert(has_named_collection_getters<spline3>);
static_assert(has_named_collection_getters<arr_spline2>);
static_assert(has_named_collection_getters<arr_spline3>);
static_assert(!has_legacy_control_points_getter<spline2>);
static_assert(!has_legacy_control_points_getter<spline3>);
static_assert(!has_legacy_control_points_getter<arr_spline2>);
static_assert(!has_legacy_control_points_getter<arr_spline3>);
static_assert(!has_legacy_weights_getter<spline2>);
static_assert(!has_legacy_weights_getter<spline3>);
static_assert(!has_legacy_weights_getter<arr_spline2>);
static_assert(!has_legacy_weights_getter<arr_spline3>);
static_assert(!has_legacy_knots_getter<spline2>);
static_assert(!has_legacy_knots_getter<spline3>);
static_assert(!has_legacy_knots_getter<arr_spline2>);
static_assert(!has_legacy_knots_getter<arr_spline3>);
static_assert(!has_legacy_control_point_getter<spline2>);
static_assert(!has_legacy_control_point_getter<spline3>);
static_assert(!has_legacy_control_point_getter<arr_spline2>);
static_assert(!has_legacy_control_point_getter<arr_spline3>);
static_assert(!has_legacy_weight_getter<spline2>);
static_assert(!has_legacy_weight_getter<spline3>);
static_assert(!has_legacy_weight_getter<arr_spline2>);
static_assert(!has_legacy_weight_getter<arr_spline3>);
static_assert(!has_legacy_knot_getter<spline2>);
static_assert(!has_legacy_knot_getter<spline3>);
static_assert(!has_legacy_knot_getter<arr_spline2>);
static_assert(!has_legacy_knot_getter<arr_spline3>);
static_assert(std::same_as<
              decltype(std::declval<const spline2&>().get_control_points()),
              const std::vector<nurbspath::point2<real>>&>);
static_assert(std::same_as<
              decltype(std::declval<const spline2&>().get_weights()),
              const std::vector<real>&>);
static_assert(std::same_as<
              decltype(std::declval<const spline2&>().get_knots()),
              const std::vector<real>&>);
static_assert(std::same_as<
              decltype(std::declval<const spline3&>().get_control_points()),
              const std::vector<nurbspath::point3<real>>&>);
static_assert(std::same_as<
              decltype(std::declval<const spline3&>().get_weights()),
              const std::vector<real>&>);
static_assert(std::same_as<
              decltype(std::declval<const spline3&>().get_knots()),
              const std::vector<real>&>);
static_assert(noexcept(std::declval<const spline2&>().get_control_points()));
static_assert(noexcept(std::declval<const spline2&>().get_weights()));
static_assert(noexcept(std::declval<const spline2&>().get_knots()));
static_assert(noexcept(std::declval<const spline3&>().get_control_points()));
static_assert(noexcept(std::declval<const spline3&>().get_weights()));
static_assert(noexcept(std::declval<const spline3&>().get_knots()));
static_assert(std::same_as<
              decltype(std::declval<const arr_spline2&>().get_control_points()),
              std::vector<nurbspath::point2<real>>>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline2&>().get_weights()),
              std::vector<real>>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline2&>().get_knots()),
              std::vector<real>>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline3&>().get_control_points()),
              std::vector<nurbspath::point3<real>>>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline3&>().get_weights()),
              std::vector<real>>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline3&>().get_knots()),
              std::vector<real>>);
static_assert(
    !noexcept(std::declval<const arr_spline2&>().get_control_points()));
static_assert(!noexcept(std::declval<const arr_spline2&>().get_weights()));
static_assert(!noexcept(std::declval<const arr_spline2&>().get_knots()));
static_assert(
    !noexcept(std::declval<const arr_spline3&>().get_control_points()));
static_assert(!noexcept(std::declval<const arr_spline3&>().get_weights()));
static_assert(!noexcept(std::declval<const arr_spline3&>().get_knots()));
static_assert(std::same_as<
              decltype(std::declval<const arr_spline2&>().to_nurbs_spline()),
              spline2>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline3&>().to_nurbs_spline()),
              spline3>);
static_assert(std::same_as<
              decltype(
                  std::declval<const arr_spline2&>().get_control_point(0)),
              nurbspath::point2<real>>);
static_assert(std::same_as<
              decltype(
                  std::declval<const arr_spline3&>().get_control_point(0)),
              nurbspath::point3<real>>);
static_assert(std::same_as<
              decltype(std::declval<const spline2&>().get_control_point(0)),
              const nurbspath::point2<real>&>);
static_assert(std::same_as<
              decltype(std::declval<const spline3&>().get_control_point(0)),
              const nurbspath::point3<real>&>);
static_assert(std::same_as<
              decltype(std::declval<const spline2&>().get_weight(0)),
              real>);
static_assert(std::same_as<
              decltype(std::declval<const spline2&>().get_knot(0)), real>);
static_assert(std::same_as<
              decltype(std::declval<const spline3&>().get_weight(0)),
              real>);
static_assert(std::same_as<
              decltype(std::declval<const spline3&>().get_knot(0)), real>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline2&>().get_weight(0)),
              real>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline2&>().get_knot(0)),
              real>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline3&>().get_weight(0)),
              real>);
static_assert(std::same_as<
              decltype(std::declval<const arr_spline3&>().get_knot(0)),
              real>);
static_assert(
    !noexcept(std::declval<const spline2&>().get_control_point(0)));
static_assert(!noexcept(std::declval<const spline2&>().get_weight(0)));
static_assert(!noexcept(std::declval<const spline2&>().get_knot(0)));
static_assert(
    !noexcept(std::declval<const spline3&>().get_control_point(0)));
static_assert(!noexcept(std::declval<const spline3&>().get_weight(0)));
static_assert(!noexcept(std::declval<const spline3&>().get_knot(0)));
static_assert(
    !noexcept(std::declval<const arr_spline2&>().get_control_point(0)));
static_assert(!noexcept(std::declval<const arr_spline2&>().get_weight(0)));
static_assert(!noexcept(std::declval<const arr_spline2&>().get_knot(0)));
static_assert(
    !noexcept(std::declval<const arr_spline3&>().get_control_point(0)));
static_assert(!noexcept(std::declval<const arr_spline3&>().get_weight(0)));
static_assert(!noexcept(std::declval<const arr_spline3&>().get_knot(0)));
static_assert(!directly_addable<nurbspath::svg_canvas2<real>, arr_spline2>);
static_assert(!directly_addable<nurbspath::svg_document3<real>, arr_spline3>);

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

[[nodiscard]] bool same_values(
    const std::valarray<real>& first,
    const std::valarray<real>& second) {
    if (first.size() != second.size()) {
        return false;
    }
    for (std::size_t index = 0; index < first.size(); ++index) {
        if (first[index] != second[index]) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] std::valarray<real> prefix(
    const std::valarray<real>& source,
    std::size_t count) {
    std::valarray<real> result(real(0), count);
    for (std::size_t index = 0; index < std::min(count, source.size()); ++index) {
        result[index] = source[index];
    }
    return result;
}

[[nodiscard]] std::valarray<real> quadratic_parameters2() {
    return {
        0.0, 0.0,
        1.0, 2.0,
        3.0, 1.0,
        1.0, 2.0, 3.0,
        -1.0, -1.0, -1.0, 4.0, 4.0, 4.0};
}

[[nodiscard]] std::valarray<real> quadratic_parameters3() {
    return {
        0.0, 0.0, 0.0,
        1.0, 2.0, 3.0,
        3.0, 1.0, -2.0,
        1.0, 2.0, 3.0,
        -1.0, -1.0, -1.0, 4.0, 4.0, 4.0};
}

[[nodiscard]] spline2 rational_spline2() {
    return {
        {{0.0, 0.0}, {1.0, 3.0}, {4.0, -1.0}, {6.0, 2.0}},
        {1.0, 2.0, 0.75, 1.5},
        {-2.0, -2.0, -2.0, -2.0, 5.0, 5.0, 5.0, 5.0},
        3,
        false,
        1e-9};
}

[[nodiscard]] spline3 rational_spline3() {
    return {
        {{0.0, 0.0, 1.0},
         {1.0, 3.0, -2.0},
         {4.0, -1.0, 5.0},
         {6.0, 2.0, 3.0}},
        {1.0, 2.0, 0.75, 1.5},
        {-2.0, -2.0, -2.0, -2.0, 5.0, 5.0, 5.0, 5.0},
        3,
        false,
        1e-9};
}

void test_standard_constructor2() {
    using namespace test_support;

    const nurbspath::point2<real> start{-1.0, 2.0};
    const nurbspath::point2<real> end{3.0, 5.0};
    constexpr std::size_t control_count = 5;
    constexpr std::size_t degree = 2;
    constexpr real tolerance = 2e-10;
    const real endpoint_distance = nurbspath::distance(start, end);
    const arr_spline2 defaulted_spline(
        start, end, control_count, degree);
    check(!defaulted_spline.is_closed() &&
              defaulted_spline.tolerance() ==
                  real(64) * std::numeric_limits<real>::epsilon(),
          "2D standard constructor defaults closure and tolerance");

    arr_spline2 spline(
        start, end, control_count, degree, false, tolerance);

    check(spline.control_point_count() == control_count &&
              spline.weight_count() == control_count &&
              spline.degree() == degree && !spline.is_closed() &&
              spline.tolerance() == tolerance,
          "2D standard constructor preserves counts and open metadata");
    for (std::size_t index = 0; index < control_count; ++index) {
        const real fraction = static_cast<real>(index) /
            static_cast<real>(control_count - 1);
        check_point2(
            spline.get_control_point(index),
            nurbspath::lerp(start, end, fraction),
            0.0,
            "2D standard constructor uses exact point lerp controls");
        check(spline.get_weight(index) == 1.0,
              "2D standard constructor uses unit weights");
    }

    const std::vector<real> expected_knots{
        0.0,
        0.0,
        0.0,
        endpoint_distance / 3.0,
        2.0 * endpoint_distance / 3.0,
        endpoint_distance,
        endpoint_distance,
        endpoint_distance};
    check(spline.knot_count() == expected_knots.size(),
          "2D standard constructor creates the required knot count");
    for (std::size_t index = 0; index < expected_knots.size(); ++index) {
        check_near(
            spline.get_knot(index),
            expected_knots[index],
            1e-14,
            "2D standard constructor creates standard distance knots");
    }
    check_near(spline.s_min(), 0.0, 0.0,
               "2D standard constructor starts its domain at zero");
    check_near(spline.s_max(), endpoint_distance, 0.0,
               "2D standard constructor ends its domain at point distance");
    check_point2(spline.get_start(), start, 0.0,
                 "2D standard constructor starts at its first point");
    check_point2(spline.get_end(), end, 1e-14,
                 "2D standard constructor ends at its second point");

    constexpr float converted_tolerance = 3e-6F;
    const arr_spline2 tolerance_overload(
        start, end, control_count, degree, converted_tolerance);
    check(!tolerance_overload.is_closed() &&
              tolerance_overload.tolerance() ==
                  static_cast<real>(converted_tolerance),
          "2D standard numeric argument selects tolerance, not closure");

    check(throws_exception<std::invalid_argument>([&] {
              static_cast<void>(arr_spline2(start, end, 2, 2));
          }),
          "2D standard constructor rejects a count not above degree");
    check(throws_exception<std::invalid_argument>([&] {
              static_cast<void>(arr_spline2(start, end, 3, 0));
          }),
          "2D standard constructor rejects degree zero");
    check(throws_exception<std::invalid_argument>([&] {
              static_cast<void>(arr_spline2(start, start, 3, 2));
          }),
          "2D standard constructor rejects a zero-distance knot domain");
    check(throws_exception<std::overflow_error>([&] {
              static_cast<void>(arr_spline2(
                  start,
                  end,
                  std::numeric_limits<std::size_t>::max() / 3,
                  1));
          }),
          "2D standard constructor rejects flattened-size overflow before allocation");
    check(throws_exception<std::invalid_argument>([&] {
              static_cast<void>(
                  arr_spline2(start, end, control_count, degree, true));
          }),
          "2D standard constructor rejects an ordinary distinct closed seam");
}

void test_standard_constructor3() {
    using namespace test_support;

    const nurbspath::point3<real> start{1.0, -2.0, 3.0};
    const nurbspath::point3<real> end{4.0, 2.0, 3.0};
    constexpr std::size_t control_count = 6;
    constexpr std::size_t degree = 3;
    constexpr real tolerance = 4e-10;
    const real endpoint_distance = nurbspath::distance(start, end);
    const arr_spline3 defaulted_spline(
        start, end, control_count, degree);
    check(!defaulted_spline.is_closed() &&
              defaulted_spline.tolerance() ==
                  real(64) * std::numeric_limits<real>::epsilon(),
          "3D standard constructor defaults closure and tolerance");

    arr_spline3 spline(
        start, end, control_count, degree, false, tolerance);

    check(spline.control_point_count() == control_count &&
              spline.weight_count() == control_count &&
              spline.degree() == degree && !spline.is_closed() &&
              spline.tolerance() == tolerance,
          "3D standard constructor preserves counts and open metadata");
    for (std::size_t index = 0; index < control_count; ++index) {
        const real fraction = static_cast<real>(index) /
            static_cast<real>(control_count - 1);
        check_point(
            spline.get_control_point(index),
            nurbspath::lerp(start, end, fraction),
            0.0,
            "3D standard constructor uses exact point lerp controls");
        check(spline.get_weight(index) == 1.0,
              "3D standard constructor uses unit weights");
    }

    const std::vector<real> expected_knots{
        0.0,
        0.0,
        0.0,
        0.0,
        endpoint_distance / 3.0,
        2.0 * endpoint_distance / 3.0,
        endpoint_distance,
        endpoint_distance,
        endpoint_distance,
        endpoint_distance};
    check(spline.knot_count() == expected_knots.size(),
          "3D standard constructor creates the required knot count");
    for (std::size_t index = 0; index < expected_knots.size(); ++index) {
        check_near(
            spline.get_knot(index),
            expected_knots[index],
            1e-14,
            "3D standard constructor creates standard distance knots");
    }
    check_near(spline.s_min(), 0.0, 0.0,
               "3D standard constructor starts its domain at zero");
    check_near(spline.s_max(), endpoint_distance, 0.0,
               "3D standard constructor ends its domain at point distance");
    check_point(spline.get_start(), start, 0.0,
                "3D standard constructor starts at its first point");
    check_point(spline.get_end(), end, 1e-14,
                "3D standard constructor ends at its second point");

    constexpr long double converted_tolerance = 5e-6L;
    const arr_spline3 tolerance_overload(
        start, end, control_count, degree, converted_tolerance);
    check(!tolerance_overload.is_closed() &&
              tolerance_overload.tolerance() ==
                  static_cast<real>(converted_tolerance),
          "3D standard numeric argument selects tolerance, not closure");

    check(throws_exception<std::invalid_argument>([&] {
              static_cast<void>(arr_spline3(start, end, 3, 3));
          }),
          "3D standard constructor rejects a count not above degree");
    check(throws_exception<std::invalid_argument>([&] {
              static_cast<void>(arr_spline3(start, end, 4, 0));
          }),
          "3D standard constructor rejects degree zero");
    check(throws_exception<std::invalid_argument>([&] {
              static_cast<void>(arr_spline3(start, start, 4, 3));
          }),
          "3D standard constructor rejects a zero-distance knot domain");
    check(throws_exception<std::overflow_error>([&] {
              static_cast<void>(arr_spline3(
                  start,
                  end,
                  std::numeric_limits<std::size_t>::max() / 4,
                  1));
          }),
          "3D standard constructor rejects flattened-size overflow before allocation");
    check(throws_exception<std::invalid_argument>([&] {
              static_cast<void>(
                  arr_spline3(start, end, control_count, degree, true));
          }),
          "3D standard constructor rejects an ordinary distinct closed seam");
}

void test_flat_layout2() {
    using namespace test_support;

    const std::valarray<real> expected = quadratic_parameters2();
    constexpr float converted_tolerance = 1e-7F;
    arr_spline2 spline(expected, 2, converted_tolerance);
    check(same_values(spline.parameters, expected),
          "2D flat constructor preserves the exact scalar array");
    check(spline.number_of_parameters() == 15 && spline.degree() == 2 &&
              !spline.is_closed() &&
              spline.tolerance() == static_cast<real>(converted_tolerance),
          "2D flat constructor keeps a numeric tolerance distinct from closure");
    check(spline.control_point_count() == 3 && spline.weight_count() == 3 &&
              spline.knot_count() == 6,
          "2D flat layout derives control, weight, and knot counts");

    const auto control_range = spline.get_control_point_count_range();
    const auto weight_range = spline.get_weight_count_range();
    const auto knot_range = spline.get_knot_count_range();
    check(std::ranges::equal(
              control_range,
              std::views::iota(std::size_t(0), std::size_t(6))) &&
              std::ranges::equal(
                  weight_range,
                  std::views::iota(std::size_t(6), std::size_t(9))) &&
              std::ranges::equal(
                  knot_range,
                  std::views::iota(std::size_t(9), std::size_t(15))),
          "2D ranges expose exact flattened control-scalar, weight, and knot indices");
    check(control_range.size() == 2 * spline.control_point_count() &&
              weight_range.size() == spline.weight_count() &&
              knot_range.size() == spline.knot_count() &&
              control_range.size() + weight_range.size() +
                      knot_range.size() ==
                  spline.number_of_parameters(),
          "2D ranges partition every scalar and count coordinates individually");

    for (std::size_t index = 0; index < expected.size(); ++index) {
        check(spline.get_parameter(index) == expected[index],
              "2D parameter accessor follows public flat storage");
    }
    check(spline.parameter_name(0) == "P[0].x" &&
              spline.parameter_name(1) == "P[0].y" &&
              spline.parameter_name(5) == "P[2].y" &&
              spline.parameter_name(6) == "W[0]" &&
              spline.parameter_name(8) == "W[2]" &&
              spline.parameter_name(9) == "K[0]" &&
              spline.parameter_name(14) == "K[5]",
          "2D names mark every flattened group boundary");
    check(throws_exception<std::out_of_range>([&spline] {
              static_cast<void>(spline.get_parameter(15));
          }) &&
              throws_exception<std::out_of_range>([&spline] {
                  static_cast<void>(spline.parameter_name(15));
              }),
          "2D flattened access rejects an index at the array end");

    check_point2(spline.get_control_point(0), {0.0, 0.0}, 0.0,
                 "2D control point zero is reconstructed from coordinates");
    check_point2(spline.get_control_point(1), {1.0, 2.0}, 0.0,
                 "2D control point one is reconstructed from coordinates");
    check(spline.get_weight(1) == 2.0 && spline.get_knot(3) == 4.0,
          "2D weight and knot accessors address their flat groups");
    check(throws_exception<std::out_of_range>([&spline] {
              static_cast<void>(spline.get_control_point(3));
          }) &&
              throws_exception<std::out_of_range>([&spline] {
                  static_cast<void>(spline.get_weight(3));
              }) &&
              throws_exception<std::out_of_range>([&spline] {
                  static_cast<void>(spline.get_knot(6));
              }),
          "2D indexed getters reject one-past-end indices");
    const auto controls = spline.get_control_points();
    const auto weights = spline.get_weights();
    const auto knots = spline.get_knots();
    check(controls.size() == 3 && weights.size() == 3 && knots.size() == 6,
          "2D collection snapshots derive the correct counts");

    auto detached_point = spline.get_control_point(1);
    detached_point.x = 99.0;
    check(detached_point.x == 99.0 &&
              spline.get_control_point(1).x == 1.0 &&
              spline.parameters[2] == 1.0,
          "2D point access returns a detached value, not scalar-array aliasing");

    arr_spline2 copied = spline;
    copied.parameters[0] = 8.0;
    check(spline.parameters[0] == 0.0 && copied.parameters[0] == 8.0,
          "2D whole-object copies own independent public arrays");

    const std::valarray<real> replacement{
        5.0, 6.0,
        6.0, 8.0,
        7.0, 7.0,
        9.0, 10.0,
        1.0, 1.0, 1.0, 1.0,
        -3.0, -3.0, -3.0, 0.0, 4.0, 4.0, 4.0};
    spline.parameters = replacement;
    check(spline.number_of_parameters() == 19 &&
              spline.get_control_points().size() == 4 &&
              spline.get_weights().size() == 4 &&
              spline.get_knots().size() == 7,
          "2D public array replacement can validly change control count");
    check_near(spline.s_min(), -3.0, 0.0,
               "2D replacement updates native domain start");
    check_near(spline.s_max(), 4.0, 0.0,
               "2D replacement updates native domain end");
    check_point2(spline.get_start(), {5.0, 6.0}, 0.0,
                 "2D start is evaluated dynamically after replacement");
    check_point2(spline.get_end(), {9.0, 10.0}, 0.0,
                 "2D end is evaluated dynamically after replacement");
    check(std::ranges::equal(
              spline.get_control_point_count_range(),
              std::views::iota(std::size_t(0), std::size_t(8))) &&
              std::ranges::equal(
                  spline.get_weight_count_range(),
                  std::views::iota(std::size_t(8), std::size_t(12))) &&
              std::ranges::equal(
                  spline.get_knot_count_range(),
                  std::views::iota(std::size_t(12), std::size_t(19))),
          "2D range queries reflect a replaced public array layout");

    const std::valarray<real> before_rejected_set = spline.parameters;
    check(throws_exception<std::invalid_argument>([&spline] {
              spline.set_parameters(prefix(spline.parameters, 18));
          }) &&
              same_values(spline.parameters, before_rejected_set),
          "2D checked whole-array replacement rejects shape atomically");
}

void test_flat_layout3() {
    using namespace test_support;

    const std::valarray<real> expected = quadratic_parameters3();
    constexpr long double converted_tolerance = 1e-7L;
    arr_spline3 spline(expected, 2, converted_tolerance);
    check(same_values(spline.parameters, expected),
          "3D flat constructor preserves the exact scalar array");
    check(spline.number_of_parameters() == 18 && spline.degree() == 2 &&
              !spline.is_closed() &&
              spline.tolerance() == static_cast<real>(converted_tolerance),
          "3D flat constructor keeps a numeric tolerance distinct from closure");
    check(spline.control_point_count() == 3 && spline.weight_count() == 3 &&
              spline.knot_count() == 6,
          "3D flat layout derives control, weight, and knot counts");

    const auto control_range = spline.get_control_point_count_range();
    const auto weight_range = spline.get_weight_count_range();
    const auto knot_range = spline.get_knot_count_range();
    check(std::ranges::equal(
              control_range,
              std::views::iota(std::size_t(0), std::size_t(9))) &&
              std::ranges::equal(
                  weight_range,
                  std::views::iota(std::size_t(9), std::size_t(12))) &&
              std::ranges::equal(
                  knot_range,
                  std::views::iota(std::size_t(12), std::size_t(18))),
          "3D ranges expose exact flattened control-scalar, weight, and knot indices");
    check(control_range.size() == 3 * spline.control_point_count() &&
              weight_range.size() == spline.weight_count() &&
              knot_range.size() == spline.knot_count() &&
              control_range.size() + weight_range.size() +
                      knot_range.size() ==
                  spline.number_of_parameters(),
          "3D ranges partition every scalar and count coordinates individually");

    for (std::size_t index = 0; index < expected.size(); ++index) {
        check(spline.get_parameter(index) == expected[index],
              "3D parameter accessor follows public flat storage");
    }
    check(spline.parameter_name(0) == "P[0].x" &&
              spline.parameter_name(1) == "P[0].y" &&
              spline.parameter_name(2) == "P[0].z" &&
              spline.parameter_name(8) == "P[2].z" &&
              spline.parameter_name(9) == "W[0]" &&
              spline.parameter_name(11) == "W[2]" &&
              spline.parameter_name(12) == "K[0]" &&
              spline.parameter_name(17) == "K[5]",
          "3D names mark every flattened group boundary");
    check_point(spline.get_control_point(1), {1.0, 2.0, 3.0}, 0.0,
                "3D control points are reconstructed from scalar triples");
    check(spline.get_weight(1) == 2.0 && spline.get_knot(3) == 4.0,
          "3D weight and knot accessors address their flat groups");
    check(throws_exception<std::out_of_range>([&spline] {
              static_cast<void>(spline.get_control_point(3));
          }) &&
              throws_exception<std::out_of_range>([&spline] {
                  static_cast<void>(spline.get_weight(3));
              }) &&
              throws_exception<std::out_of_range>([&spline] {
                  static_cast<void>(spline.get_knot(6));
              }),
          "3D indexed getters reject one-past-end indices");

    auto detached_point = spline.get_control_point(1);
    detached_point.z = 99.0;
    check(detached_point.z == 99.0 &&
              spline.get_control_point(1).z == 3.0 &&
              spline.parameters[5] == 3.0,
          "3D point access returns a detached value, not scalar-array aliasing");

    arr_spline3 copied = spline;
    copied.parameters[0] = 8.0;
    check(spline.parameters[0] == 0.0 && copied.parameters[0] == 8.0,
          "3D whole-object copies own independent public arrays");

    const std::valarray<real> replacement{
        5.0, 6.0, 7.0,
        6.0, 8.0, 9.0,
        7.0, 7.0, 5.0,
        9.0, 10.0, 11.0,
        1.0, 1.0, 1.0, 1.0,
        -3.0, -3.0, -3.0, 0.0, 4.0, 4.0, 4.0};
    spline.parameters = replacement;
    check(spline.number_of_parameters() == 23 &&
              spline.get_control_points().size() == 4 &&
              spline.get_weights().size() == 4 &&
              spline.get_knots().size() == 7,
          "3D public array replacement can validly change control count");
    check_near(spline.s_min(), -3.0, 0.0,
               "3D replacement updates native domain start");
    check_near(spline.s_max(), 4.0, 0.0,
               "3D replacement updates native domain end");
    check_point(spline.get_start(), {5.0, 6.0, 7.0}, 0.0,
                "3D start is evaluated dynamically after replacement");
    check_point(spline.get_end(), {9.0, 10.0, 11.0}, 0.0,
                "3D end is evaluated dynamically after replacement");
    check(std::ranges::equal(
              spline.get_control_point_count_range(),
              std::views::iota(std::size_t(0), std::size_t(12))) &&
              std::ranges::equal(
                  spline.get_weight_count_range(),
                  std::views::iota(std::size_t(12), std::size_t(16))) &&
              std::ranges::equal(
                  spline.get_knot_count_range(),
                  std::views::iota(std::size_t(16), std::size_t(23))),
          "3D range queries reflect a replaced public array layout");

    const std::valarray<real> before_rejected_set = spline.parameters;
    check(throws_exception<std::invalid_argument>([&spline] {
              spline.set_parameters(prefix(spline.parameters, 22));
          }) &&
              same_values(spline.parameters, before_rejected_set),
          "3D checked whole-array replacement rejects shape atomically");
}

void test_pointer_parameters2() {
    using namespace test_support;

    arr_spline2 spline(quadratic_parameters2(), 2);
    const std::valarray<real> external{
        -2.0, 1.0,
        2.0, 3.0,
        5.0, -1.0,
        1.0, 1.5, 2.0,
        0.0, 0.0, 0.0, 6.0, 6.0, 6.0};
    spline.set_parameters(std::begin(external));
    check(same_values(spline.parameters, external) &&
              spline.get_start().x == -2.0 && spline.s_max() == 6.0,
          "2D pointer setter copies a complete const external buffer");

    spline.parameters[0] = -3.0;
    const std::valarray<real> before_alias = spline.parameters;
    const real* const current_storage = std::begin(spline.parameters);
    spline.set_parameters(current_storage);
    check(same_values(spline.parameters, before_alias) &&
              spline.get_start().x == -3.0,
          "2D pointer setter detaches a source aliasing current storage");

    const std::valarray<real> before_null = spline.parameters;
    check(throws_exception<std::invalid_argument>([&spline] {
              spline.set_parameters(static_cast<const real*>(nullptr));
          }) &&
              same_values(spline.parameters, before_null),
          "2D pointer setter rejects null without changing the spline");

    std::valarray<real> invalid_candidate = spline.parameters;
    invalid_candidate[6] = 0.0;
    const std::valarray<real> before_invalid = spline.parameters;
    check(throws_exception<std::invalid_argument>(
              [&spline, &invalid_candidate] {
                  spline.set_parameters(std::begin(invalid_candidate));
              }) &&
              same_values(spline.parameters, before_invalid),
          "2D pointer setter rejects an invalid candidate atomically");

    spline.parameters[6] = 0.0;
    check(throws_exception<std::invalid_argument>([&spline] {
              spline.validate();
          }),
          "2D direct mutation can leave a value-invalid definition");
    spline.set_parameters(std::begin(external));
    check(same_values(spline.parameters, external),
          "2D pointer setter repairs invalid values from a valid buffer");

    spline.parameters = prefix(external, external.size() - 1);
    const std::valarray<real> malformed_source = spline.parameters;
    const std::valarray<real> before_malformed = spline.parameters;
    check(throws_exception<std::invalid_argument>(
              [&spline, &malformed_source] {
                  spline.set_parameters(std::begin(malformed_source));
              }) &&
              same_values(spline.parameters, before_malformed),
          "2D pointer setter rejects a same-length malformed shape atomically");
}

void test_pointer_parameters3() {
    using namespace test_support;

    arr_spline3 spline(quadratic_parameters3(), 2);
    const std::valarray<real> external{
        -2.0, 1.0, 4.0,
        2.0, 3.0, -1.0,
        5.0, -1.0, 2.0,
        1.0, 1.5, 2.0,
        0.0, 0.0, 0.0, 6.0, 6.0, 6.0};
    spline.set_parameters(std::begin(external));
    check(same_values(spline.parameters, external) &&
              spline.get_start().x == -2.0 && spline.s_max() == 6.0,
          "3D pointer setter copies a complete const external buffer");

    spline.parameters[0] = -3.0;
    const std::valarray<real> before_alias = spline.parameters;
    const real* const current_storage = std::begin(spline.parameters);
    spline.set_parameters(current_storage);
    check(same_values(spline.parameters, before_alias) &&
              spline.get_start().x == -3.0,
          "3D pointer setter detaches a source aliasing current storage");

    const std::valarray<real> before_null = spline.parameters;
    check(throws_exception<std::invalid_argument>([&spline] {
              spline.set_parameters(static_cast<const real*>(nullptr));
          }) &&
              same_values(spline.parameters, before_null),
          "3D pointer setter rejects null without changing the spline");

    std::valarray<real> invalid_candidate = spline.parameters;
    invalid_candidate[9] = 0.0;
    const std::valarray<real> before_invalid = spline.parameters;
    check(throws_exception<std::invalid_argument>(
              [&spline, &invalid_candidate] {
                  spline.set_parameters(std::begin(invalid_candidate));
              }) &&
              same_values(spline.parameters, before_invalid),
          "3D pointer setter rejects an invalid candidate atomically");

    spline.parameters[9] = 0.0;
    check(throws_exception<std::invalid_argument>([&spline] {
              spline.validate();
          }),
          "3D direct mutation can leave a value-invalid definition");
    spline.set_parameters(std::begin(external));
    check(same_values(spline.parameters, external),
          "3D pointer setter repairs invalid values from a valid buffer");

    spline.parameters = prefix(external, external.size() - 1);
    const std::valarray<real> malformed_source = spline.parameters;
    const std::valarray<real> before_malformed = spline.parameters;
    check(throws_exception<std::invalid_argument>(
              [&spline, &malformed_source] {
                  spline.set_parameters(std::begin(malformed_source));
              }) &&
              same_values(spline.parameters, before_malformed),
          "3D pointer setter rejects a same-length malformed shape atomically");
}

void test_validation2() {
    using namespace test_support;

    const std::valarray<real> valid = quadratic_parameters2();
    check(throws_exception<std::invalid_argument>([&valid] {
              static_cast<void>(arr_spline2(prefix(valid, 14), 2));
          }),
          "2D flat constructor rejects a length with no integral control count");
    check(throws_exception<std::invalid_argument>([&valid] {
              static_cast<void>(arr_spline2(valid, 0));
          }),
          "2D flat constructor rejects degree zero");
    check(throws_exception<std::invalid_argument>([&valid] {
              static_cast<void>(arr_spline2(valid, 2, 0.0));
          }),
          "2D flat constructor rejects nonpositive tolerance");

    std::valarray<real> bad = valid;
    bad[0] = std::numeric_limits<real>::quiet_NaN();
    check(throws_exception<std::invalid_argument>([&bad] {
              static_cast<void>(arr_spline2(bad, 2));
          }),
          "2D flat constructor rejects a nonfinite coordinate");
    bad = valid;
    bad[6] = 0.0;
    check(throws_exception<std::invalid_argument>([&bad] {
              static_cast<void>(arr_spline2(bad, 2));
          }),
          "2D flat constructor rejects a nonpositive weight");
    bad = valid;
    bad[12] = -2.0;
    check(throws_exception<std::invalid_argument>([&bad] {
              static_cast<void>(arr_spline2(bad, 2));
          }),
          "2D flat constructor rejects decreasing knots");

    arr_spline2 spline(valid, 2);
    spline.parameters = prefix(valid, 14);
    check(throws_exception<std::invalid_argument>([&spline] {
              static_cast<void>(spline.evaluate(0.0));
          }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.to_nurbs_spline());
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline2(spline));
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_control_point(0));
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_weight(0));
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_knot(0));
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_control_point_count_range());
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_weight_count_range());
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_knot_count_range());
              }),
          "2D geometry, both conversions, getters, and ranges reject malformed public shape");
    spline.parameters = valid;
    spline.parameters[6] = 0.0;
    check(throws_exception<std::invalid_argument>([&spline] {
              static_cast<void>(spline.first_derivative(0.0));
          }),
          "2D geometry rejects an invalid directly edited weight");
    spline.parameters = valid;
    spline.parameters[12] = -2.0;
    check(throws_exception<std::invalid_argument>([&spline] {
              static_cast<void>(spline.get_start());
          }),
          "2D dynamic endpoint access validates directly edited knots");
}

void test_validation3() {
    using namespace test_support;

    const std::valarray<real> valid = quadratic_parameters3();
    check(throws_exception<std::invalid_argument>([&valid] {
              static_cast<void>(arr_spline3(prefix(valid, 17), 2));
          }),
          "3D flat constructor rejects a length with no integral control count");
    check(throws_exception<std::invalid_argument>([&valid] {
              static_cast<void>(arr_spline3(valid, 3));
          }),
          "3D flat constructor rejects degree at the control count");

    std::valarray<real> bad = valid;
    bad[4] = std::numeric_limits<real>::infinity();
    check(throws_exception<std::invalid_argument>([&bad] {
              static_cast<void>(arr_spline3(bad, 2));
          }),
          "3D flat constructor rejects a nonfinite coordinate");
    bad = valid;
    bad[10] = -1.0;
    check(throws_exception<std::invalid_argument>([&bad] {
              static_cast<void>(arr_spline3(bad, 2));
          }),
          "3D flat constructor rejects a nonpositive weight");
    bad = valid;
    bad[15] = -2.0;
    check(throws_exception<std::invalid_argument>([&bad] {
              static_cast<void>(arr_spline3(bad, 2));
          }),
          "3D flat constructor rejects decreasing knots");

    arr_spline3 spline(valid, 2);
    spline.parameters = prefix(valid, 17);
    check(throws_exception<std::invalid_argument>([&spline] {
              static_cast<void>(spline.evaluate(0.0));
          }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.to_nurbs_spline());
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline3(spline));
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_control_point(0));
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_weight(0));
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_knot(0));
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_control_point_count_range());
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_weight_count_range());
              }) &&
              throws_exception<std::invalid_argument>([&spline] {
                  static_cast<void>(spline.get_knot_count_range());
              }),
          "3D geometry, both conversions, getters, and ranges reject malformed public shape");
    spline.parameters = valid;
    spline.parameters[10] = 0.0;
    check(throws_exception<std::invalid_argument>([&spline] {
              static_cast<void>(spline.third_derivative(0.0));
          }),
          "3D geometry rejects an invalid directly edited weight");
    spline.parameters = valid;
    spline.parameters[15] = -2.0;
    check(throws_exception<std::invalid_argument>([&spline] {
              static_cast<void>(spline.get_end());
          }),
          "3D dynamic endpoint access validates directly edited knots");
}

void test_evaluation_and_conversion2() {
    using namespace test_support;

    const spline2 ordinary = rational_spline2();
    const arr_spline2 array(ordinary);
    check(array.degree() == ordinary.degree() &&
              array.tolerance() == ordinary.tolerance() &&
              array.is_closed() == ordinary.is_closed() &&
              array.s_min() == ordinary.s_min() &&
              array.s_max() == ordinary.s_max(),
          "2D explicit conversion from ordinary spline preserves metadata");
    const std::valarray<real> expected{
        0.0, 0.0, 1.0, 3.0, 4.0, -1.0, 6.0, 2.0,
        1.0, 2.0, 0.75, 1.5,
        -2.0, -2.0, -2.0, -2.0, 5.0, 5.0, 5.0, 5.0};
    check(same_values(array.parameters, expected),
          "2D ordinary conversion uses the documented flat order");

    for (const real s : {-2.0, -0.25, 2.5, 5.0}) {
        const auto array_derivatives = array.derivatives_at(s);
        const auto ordinary_derivatives = ordinary.derivatives_at(s);
        check_point2(array.evaluate(s), ordinary.evaluate(s), 2e-12,
                     "2D array evaluation matches ordinary evaluation");
        check_point2(array.point_at(s), ordinary.point_at(s), 2e-12,
                     "2D array point_at matches ordinary point_at");
        check_point2(array_derivatives.point, ordinary_derivatives.point, 2e-12,
                     "2D derivative bundle position matches ordinary spline");
        check(array_derivatives.first.approximately_equal(
                  ordinary_derivatives.first, 2e-12) &&
                  array_derivatives.second.approximately_equal(
                      ordinary_derivatives.second, 2e-12) &&
                  array.first_derivative(s).approximately_equal(
                      ordinary.first_derivative(s), 2e-12) &&
                  array.second_derivative(s).approximately_equal(
                      ordinary.second_derivative(s), 2e-12) &&
                  array.third_derivative(s).approximately_equal(
                      ordinary.third_derivative(s), 2e-12),
              "2D array analytic derivatives match the ordinary spline");
    }
    check(array.tangent(1.0).approximately_equal(ordinary.tangent(1.0), 2e-12),
          "2D array tangent matches the ordinary spline");
    check_near(
        array.approximate_arc_length(37),
        ordinary.approximate_arc_length(37),
        2e-12,
        "2D array arc-length approximation matches the ordinary spline");
    check(throws_exception<std::invalid_argument>([&array] {
              static_cast<void>(array.approximate_arc_length(0));
          }),
          "2D array arc-length approximation rejects zero segments");

    const spline2 restored(array);
    const spline2 named_restored = array.to_nurbs_spline();
    const arr_spline2 round_trip(restored);
    const arr_spline2 named_round_trip(named_restored);
    check(restored.degree() == ordinary.degree() &&
              restored.tolerance() == ordinary.tolerance() &&
              restored.is_closed() == ordinary.is_closed() &&
              same_values(round_trip.parameters, array.parameters) &&
              same_values(named_round_trip.parameters, array.parameters),
          "2D constructor and named round trips preserve every field");

    arr_spline2 mutable_array = array;
    const spline2 detached_ordinary(mutable_array);
    const nurbspath::point2<real> detached_point =
        detached_ordinary.get_control_point(0);
    mutable_array.parameters[0] += 10.0;
    check_point2(detached_ordinary.get_control_point(0), detached_point, 0.0,
                 "2D array-to-ordinary construction detaches storage");

    spline2 mutable_ordinary = ordinary;
    const arr_spline2 detached_array(mutable_ordinary);
    mutable_ordinary.set_control_point(0, {10.0, 20.0});
    check_point2(detached_array.get_control_point(0),
                 ordinary.get_control_point(0), 0.0,
                 "2D ordinary-to-array construction detaches storage");

    const spline2 multi_span(
        {{0.0, 0.0}, {1.0, 2.0}, {2.0, -1.0}, {4.0, 3.0}, {6.0, 1.0}},
        {1.0, 0.6, 2.0, 1.25, 0.8},
        {0.0, 0.0, 0.0, 0.75, 2.25, 4.0, 4.0, 4.0},
        2,
        false,
        1e-10);
    const arr_spline2 multi_span_array(multi_span);
    for (const real s : {0.0, 0.25, 0.75, 1.5, 2.25, 3.5, 4.0}) {
        check_point2(multi_span_array.evaluate(s), multi_span.evaluate(s), 2e-11,
                     "2D rational multi-span evaluation matches");
        check(multi_span_array.first_derivative(s).approximately_equal(
                  multi_span.first_derivative(s), 2e-10) &&
                  multi_span_array.second_derivative(s).approximately_equal(
                      multi_span.second_derivative(s), 2e-10) &&
                  multi_span_array.third_derivative(s).approximately_equal(
                      multi_span.third_derivative(s), 2e-9),
              "2D rational multi-span derivatives match");
    }

    const spline2 rational_linear(
        {{0.0, 0.0}, {1.0, 2.0}, {3.0, -1.0}, {5.0, 1.0}},
        {1.0, 2.0, 0.5, 1.5},
        {-1.0, -1.0, 0.5, 2.0, 3.0, 3.0},
        1);
    const arr_spline2 rational_linear_array(rational_linear);
    for (const real s : {-1.0, 0.0, 1.0, 2.5, 3.0}) {
        check_point2(
            rational_linear_array.evaluate(s), rational_linear.evaluate(s),
            2e-12, "2D rational degree-one evaluation matches");
        check(rational_linear_array.third_derivative(s).approximately_equal(
                  rational_linear.third_derivative(s), 2e-9),
              "2D rational degree-one third derivative matches");
    }
}

void test_evaluation_and_conversion3() {
    using namespace test_support;

    const spline3 ordinary = rational_spline3();
    const arr_spline3 array(ordinary);
    check(array.degree() == ordinary.degree() &&
              array.tolerance() == ordinary.tolerance() &&
              array.is_closed() == ordinary.is_closed() &&
              array.s_min() == ordinary.s_min() &&
              array.s_max() == ordinary.s_max(),
          "3D explicit conversion from ordinary spline preserves metadata");
    const std::valarray<real> expected{
        0.0, 0.0, 1.0,
        1.0, 3.0, -2.0,
        4.0, -1.0, 5.0,
        6.0, 2.0, 3.0,
        1.0, 2.0, 0.75, 1.5,
        -2.0, -2.0, -2.0, -2.0, 5.0, 5.0, 5.0, 5.0};
    check(same_values(array.parameters, expected),
          "3D ordinary conversion uses the documented flat order");

    for (const real s : {-2.0, -0.25, 2.5, 5.0}) {
        const auto array_derivatives = array.derivatives_at(s);
        const auto ordinary_derivatives = ordinary.derivatives_at(s);
        check_point(array.evaluate(s), ordinary.evaluate(s), 2e-12,
                    "3D array evaluation matches ordinary evaluation");
        check_point(array.point_at(s), ordinary.point_at(s), 2e-12,
                    "3D array point_at matches ordinary point_at");
        check_point(array_derivatives.point, ordinary_derivatives.point, 2e-12,
                    "3D derivative bundle position matches ordinary spline");
        check(array_derivatives.first.approximately_equal(
                  ordinary_derivatives.first, 2e-12) &&
                  array_derivatives.second.approximately_equal(
                      ordinary_derivatives.second, 2e-12) &&
                  array.first_derivative(s).approximately_equal(
                      ordinary.first_derivative(s), 2e-12) &&
                  array.second_derivative(s).approximately_equal(
                      ordinary.second_derivative(s), 2e-12) &&
                  array.third_derivative(s).approximately_equal(
                      ordinary.third_derivative(s), 2e-12),
              "3D array analytic derivatives match the ordinary spline");
    }
    check(array.tangent(1.0).approximately_equal(ordinary.tangent(1.0), 2e-12),
          "3D array tangent matches the ordinary spline");
    check_near(
        array.approximate_arc_length(37),
        ordinary.approximate_arc_length(37),
        2e-12,
        "3D array arc-length approximation matches the ordinary spline");
    check(throws_exception<std::invalid_argument>([&array] {
              static_cast<void>(array.approximate_arc_length(0));
          }),
          "3D array arc-length approximation rejects zero segments");

    const spline3 restored(array);
    const spline3 named_restored = array.to_nurbs_spline();
    const arr_spline3 round_trip(restored);
    const arr_spline3 named_round_trip(named_restored);
    check(restored.degree() == ordinary.degree() &&
              restored.tolerance() == ordinary.tolerance() &&
              restored.is_closed() == ordinary.is_closed() &&
              same_values(round_trip.parameters, array.parameters) &&
              same_values(named_round_trip.parameters, array.parameters),
          "3D constructor and named round trips preserve every field");

    arr_spline3 mutable_array = array;
    const spline3 detached_ordinary(mutable_array);
    const nurbspath::point3<real> detached_point =
        detached_ordinary.get_control_point(0);
    mutable_array.parameters[0] += 10.0;
    check_point(detached_ordinary.get_control_point(0), detached_point, 0.0,
                "3D array-to-ordinary construction detaches storage");

    spline3 mutable_ordinary = ordinary;
    const arr_spline3 detached_array(mutable_ordinary);
    mutable_ordinary.set_control_point(0, {10.0, 20.0, 30.0});
    check_point(detached_array.get_control_point(0),
                ordinary.get_control_point(0), 0.0,
                "3D ordinary-to-array construction detaches storage");

    const spline3 multi_span(
        {{0.0, 0.0, 1.0},
         {1.0, 2.0, -1.0},
         {2.0, -1.0, 4.0},
         {4.0, 3.0, 2.0},
         {6.0, 1.0, -2.0}},
        {1.0, 0.6, 2.0, 1.25, 0.8},
        {0.0, 0.0, 0.0, 0.75, 2.25, 4.0, 4.0, 4.0},
        2,
        false,
        1e-10);
    const arr_spline3 multi_span_array(multi_span);
    for (const real s : {0.0, 0.25, 0.75, 1.5, 2.25, 3.5, 4.0}) {
        check_point(multi_span_array.evaluate(s), multi_span.evaluate(s), 2e-11,
                    "3D rational multi-span evaluation matches");
        check(multi_span_array.first_derivative(s).approximately_equal(
                  multi_span.first_derivative(s), 2e-10) &&
                  multi_span_array.second_derivative(s).approximately_equal(
                      multi_span.second_derivative(s), 2e-10) &&
                  multi_span_array.third_derivative(s).approximately_equal(
                      multi_span.third_derivative(s), 2e-9),
              "3D rational multi-span derivatives match");
    }

    const spline3 rational_linear(
        {{0.0, 0.0, 0.0},
         {1.0, 2.0, 3.0},
         {3.0, -1.0, 2.0},
         {5.0, 1.0, -2.0}},
        {1.0, 2.0, 0.5, 1.5},
        {-1.0, -1.0, 0.5, 2.0, 3.0, 3.0},
        1);
    const arr_spline3 rational_linear_array(rational_linear);
    for (const real s : {-1.0, 0.0, 1.0, 2.5, 3.0}) {
        check_point(
            rational_linear_array.evaluate(s), rational_linear.evaluate(s),
            2e-12, "3D rational degree-one evaluation matches");
        check(rational_linear_array.third_derivative(s).approximately_equal(
                  rational_linear.third_derivative(s), 2e-9),
              "3D rational degree-one third derivative matches");
    }
}

void test_curvature2() {
    using namespace test_support;

    const arr_spline2 line(make_line2({-2.0, 3.0}, {4.0, -1.0}, 2.0, 9.0));
    check_near(line.curvature(5.0), 0.0, 0.0,
               "2D array line has zero curvature");

    const real root_half = std::sqrt(0.5);
    const arr_spline2 unit_circle(spline2(
        {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        2));
    for (const real s : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        check_near(unit_circle.curvature(s), 1.0, 2e-12,
                   "2D array rational unit circle has unit curvature");
    }

    const arr_spline2 rescaled_circle(spline2(
        {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}},
        {1.0, root_half, 1.0},
        {2.0, 2.0, 2.0, 8.0, 8.0, 8.0},
        2));
    check_near(rescaled_circle.curvature(5.0), unit_circle.curvature(0.5),
               2e-12,
               "2D array curvature is invariant under parameter rescaling");

    const arr_spline2 narrow_circle(spline2(
        {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1e-104, 1e-104, 1e-104},
        2));
    check_near(narrow_circle.curvature(5e-105), 1.0, 2e-12,
               "2D array curvature remains stable on a narrow parameter domain");

    const arr_spline2 wide_circle(spline2(
        {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1e104, 1e104, 1e104},
        2));
    check_near(wide_circle.curvature(5e103, 0.0), 1.0, 2e-12,
               "2D array curvature remains stable on a wide parameter domain");

    const auto derivatives = unit_circle.derivatives_at(0.35);
    const real speed = derivatives.first.length();
    const real expected =
        std::abs(derivatives.first.cross(derivatives.second)) /
        (speed * speed * speed);
    check_near(unit_circle.curvature(0.35), expected, 2e-15,
               "2D array curvature uses the analytic derivative magnitude");

    const arr_spline2 stationary(spline2(
        {{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}},
        {1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        2));
    check(throws_exception<std::domain_error>([&stationary] {
              static_cast<void>(stationary.curvature(0.0));
          }) &&
              throws_exception<std::domain_error>([&line] {
                  static_cast<void>(line.curvature(5.0, 2.0));
              }),
          "2D array curvature rejects stationary and tolerance-small tangents");

    check(throws_exception<std::invalid_argument>([&unit_circle] {
              static_cast<void>(unit_circle.curvature(0.5, -1.0));
          }) &&
              throws_exception<std::invalid_argument>([&unit_circle] {
                  static_cast<void>(unit_circle.curvature(
                      0.5, std::numeric_limits<real>::infinity()));
              }) &&
              throws_exception<std::invalid_argument>([&unit_circle] {
                  static_cast<void>(unit_circle.curvature(
                      0.5, std::numeric_limits<real>::quiet_NaN()));
              }),
          "2D array curvature rejects invalid tangent tolerances");
    check(throws_exception<std::out_of_range>([&unit_circle] {
              static_cast<void>(unit_circle.curvature(2.0));
          }),
          "2D array curvature enforces the active parameter domain");

    arr_spline2 malformed = unit_circle;
    malformed.parameters = prefix(
        malformed.parameters, malformed.parameters.size() - 1);
    check(throws_exception<std::invalid_argument>([&malformed] {
              static_cast<void>(malformed.curvature(0.5));
          }),
          "2D array curvature validates public storage before evaluation");
}

void test_curvature3() {
    using namespace test_support;

    const arr_spline3 line(make_line(
        {-2.0, 3.0, 1.0}, {4.0, -1.0, 5.0}, 2.0, 9.0));
    check_near(line.curvature(5.0), 0.0, 0.0,
               "3D array line has zero curvature");

    const real root_half = std::sqrt(0.5);
    const arr_spline3 unit_circle(spline3(
        {{1.0, 0.0, 4.0}, {1.0, 1.0, 4.0}, {0.0, 1.0, 4.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        2));
    for (const real s : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        check_near(unit_circle.curvature(s), 1.0, 2e-12,
                   "3D array rational unit circle has unit curvature");
    }

    const arr_spline3 rescaled_circle(spline3(
        {{1.0, 0.0, 4.0}, {1.0, 1.0, 4.0}, {0.0, 1.0, 4.0}},
        {1.0, root_half, 1.0},
        {2.0, 2.0, 2.0, 8.0, 8.0, 8.0},
        2));
    check_near(rescaled_circle.curvature(5.0), unit_circle.curvature(0.5),
               2e-12,
               "3D array curvature is invariant under parameter rescaling");

    const arr_spline3 narrow_circle(spline3(
        {{1.0, 0.0, 4.0}, {1.0, 1.0, 4.0}, {0.0, 1.0, 4.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1e-104, 1e-104, 1e-104},
        2));
    check_near(narrow_circle.curvature(5e-105), 1.0, 2e-12,
               "3D array curvature remains stable on a narrow parameter domain");

    const arr_spline3 wide_circle(spline3(
        {{1.0, 0.0, 4.0}, {1.0, 1.0, 4.0}, {0.0, 1.0, 4.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1e104, 1e104, 1e104},
        2));
    check_near(wide_circle.curvature(5e103, 0.0), 1.0, 2e-12,
               "3D array curvature remains stable on a wide parameter domain");

    const auto derivatives = unit_circle.derivatives_at(0.35);
    const real speed = derivatives.first.length();
    const real expected = derivatives.first.cross(derivatives.second).length() /
                          (speed * speed * speed);
    check_near(unit_circle.curvature(0.35), expected, 2e-15,
               "3D array curvature uses the analytic derivative magnitude");

    const arr_spline3 stationary(spline3(
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
        {1.0, 1.0, 1.0},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        2));
    check(throws_exception<std::domain_error>([&stationary] {
              static_cast<void>(stationary.curvature(0.0));
          }) &&
              throws_exception<std::domain_error>([&line] {
                  static_cast<void>(line.curvature(5.0, 2.0));
              }),
          "3D array curvature rejects stationary and tolerance-small tangents");

    check(throws_exception<std::invalid_argument>([&unit_circle] {
              static_cast<void>(unit_circle.curvature(0.5, -1.0));
          }) &&
              throws_exception<std::invalid_argument>([&unit_circle] {
                  static_cast<void>(unit_circle.curvature(
                      0.5, std::numeric_limits<real>::infinity()));
              }) &&
              throws_exception<std::invalid_argument>([&unit_circle] {
                  static_cast<void>(unit_circle.curvature(
                      0.5, std::numeric_limits<real>::quiet_NaN()));
              }),
          "3D array curvature rejects invalid tangent tolerances");
    check(throws_exception<std::out_of_range>([&unit_circle] {
              static_cast<void>(unit_circle.curvature(2.0));
          }),
          "3D array curvature enforces the active parameter domain");

    arr_spline3 malformed = unit_circle;
    malformed.parameters = prefix(
        malformed.parameters, malformed.parameters.size() - 1);
    check(throws_exception<std::invalid_argument>([&malformed] {
              static_cast<void>(malformed.curvature(0.5));
          }),
          "3D array curvature validates public storage before evaluation");
}

void test_checked_mutation2() {
    using namespace test_support;

    arr_spline2 spline(quadratic_parameters2(), 2);
    check(spline.set_parameter(0, -2.0) && spline.parameters[0] == -2.0 &&
              spline.get_start().x == -2.0,
          "2D scalar setter validates and exposes a committed coordinate");
    std::valarray<real> before = spline.parameters;
    check(!spline.set_parameter(6, 0.0) &&
              same_values(spline.parameters, before) &&
              !spline.set_parameter(spline.number_of_parameters(), 1.0),
          "2D scalar setter rejects invalid values and indices atomically");

    spline.set_control_point(1, {2.0, 3.0});
    spline.set_weight(1, 1.25);
    spline.set_knots({-4.0, -4.0, -4.0, 7.0, 7.0, 7.0});
    spline.set_tolerance(1e-7);
    check_point2(spline.get_control_point(1), {2.0, 3.0}, 0.0,
                 "2D checked point setter commits scalar coordinates");
    check(spline.get_weight(1) == 1.25 && spline.s_min() == -4.0 &&
              spline.s_max() == 7.0 && spline.tolerance() == 1e-7,
          "2D checked weight, knot, and tolerance setters commit together");

    before = spline.parameters;
    check(throws_exception<std::invalid_argument>([&spline] {
              spline.set_control_point(
                  1, {std::numeric_limits<real>::quiet_NaN(), 0.0});
          }) &&
              same_values(spline.parameters, before),
          "2D checked point failure leaves public storage unchanged");
    check(throws_exception<std::invalid_argument>([&spline] {
              spline.set_weight(1, -1.0);
          }) &&
              same_values(spline.parameters, before),
          "2D checked weight failure leaves public storage unchanged");

    spline.set_definition(
        {{1.0, 1.0}, {2.0, 4.0}, {3.0, 2.0}, {5.0, 6.0}},
        {1.0, 1.0, 1.0, 1.0},
        {2.0, 2.0, 2.0, 3.0, 8.0, 8.0, 8.0},
        false,
        2e-8);
    check(spline.degree() == 2 &&
              spline.get_control_points().size() == 4 &&
              spline.parameters.size() == 19 && spline.tolerance() == 2e-8,
          "2D bulk setter changes array shape while retaining degree");
}

void test_checked_mutation3() {
    using namespace test_support;

    arr_spline3 spline(
        std::vector<nurbspath::point3<real>>{
            {0.0, 0.0, 0.0},
            {1.0, 2.0, 1.0},
            {2.0, 3.0, 2.0},
            {3.0, 1.0, 3.0},
            {4.0, 0.0, 4.0}},
        std::vector<real>{1.0, 1.0, 1.0, 1.0, 1.0},
        std::vector<real>{0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0},
        2,
        false,
        1e-9);
    spline.set_control_point(1, {1.0, 4.0, -1.0});
    spline.set_weight(1, 2.0);
    check_point(spline.get_control_point(1), {1.0, 4.0, -1.0}, 0.0,
                "3D checked point setter commits scalar coordinates");
    check(spline.get_weight(1) == 2.0,
          "3D checked weight setter commits its scalar");

    spline.set_standard_knots(-3.0, 6.0);
    check(spline.get_knot(0) == -3.0 && spline.get_knot(2) == -3.0 &&
              spline.get_knot(5) == 6.0 && spline.get_knot(7) == 6.0 &&
              spline.s_min() == -3.0 && spline.s_max() == 6.0,
          "3D standard-knot setter creates clamped endpoint blocks");
    check_near(spline.get_knot(3), 0.0, 1e-14,
               "3D standard-knot setter creates first uniform interior knot");
    check_near(spline.get_knot(4), 3.0, 1e-14,
               "3D standard-knot setter creates second uniform interior knot");
    spline.set_standard_knots(9.0);
    check(spline.get_knot(0) == 0.0 && spline.get_knot(7) == 9.0,
          "3D one-bound standard-knot setter starts at zero");
    check_near(spline.get_knot(3), 3.0, 1e-14,
               "3D zero-based standard knots use uniform first interior knot");
    check_near(spline.get_knot(4), 6.0, 1e-14,
               "3D zero-based standard knots use uniform second interior knot");
    const std::valarray<real> before = spline.parameters;
    check(throws_exception<std::invalid_argument>([&spline] {
              spline.set_standard_knots(2.0, 2.0);
          }) &&
              same_values(spline.parameters, before),
          "3D invalid standard-knot domain leaves storage unchanged");
}

void test_closed_curves() {
    using namespace test_support;

    arr_spline2 closed2(
        std::vector<nurbspath::point2<real>>{
            {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}},
        std::vector<real>{1.0, 1.0, 1.0, 1.0},
        std::vector<real>{0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
        1,
        true,
        1e-10);
    check(closed2.is_closed(), "2D array spline records closure");
    check_point2(closed2.get_start(), closed2.get_end(), 0.0,
                 "2D array spline computes coincident dynamic endpoints");
    const spline2 closed2_ordinary(closed2);
    const arr_spline2 closed2_round_trip(closed2_ordinary);
    check(closed2_round_trip.is_closed() &&
              closed2_round_trip.tolerance() == closed2.tolerance() &&
              same_values(closed2_round_trip.parameters, closed2.parameters),
          "2D closed named round trip preserves closure and all state");
    std::valarray<real> before2 = closed2.parameters;
    check(throws_exception<std::invalid_argument>([&closed2] {
              closed2.set_control_point(3, {2.0, 2.0});
          }) &&
              same_values(closed2.parameters, before2),
          "2D checked mutation cannot break a closed seam");
    closed2.parameters[6] = 2.0;
    check(throws_exception<std::invalid_argument>([&closed2] {
              static_cast<void>(closed2.evaluate(1.5));
          }) &&
              throws_exception<std::invalid_argument>([&closed2] {
                  static_cast<void>(spline2(closed2));
              }),
          "2D geometry and constructor validate a directly broken closed seam");
    closed2.parameters = before2;

    arr_spline3 closed3(
        std::vector<nurbspath::point3<real>>{
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 1.0},
            {0.0, 1.0, 2.0},
            {0.0, 0.0, 0.0}},
        std::vector<real>{1.0, 1.0, 1.0, 1.0},
        std::vector<real>{2.0, 2.0, 3.0, 4.0, 5.0, 5.0},
        1,
        true,
        1e-10);
    check(closed3.is_closed(), "3D array spline records closure");
    check_point(closed3.get_start(), closed3.get_end(), 0.0,
                "3D array spline computes coincident dynamic endpoints");
    const spline3 closed3_ordinary(closed3);
    const arr_spline3 closed3_round_trip(closed3_ordinary);
    check(closed3_round_trip.is_closed() &&
              closed3_round_trip.tolerance() == closed3.tolerance() &&
              same_values(closed3_round_trip.parameters, closed3.parameters),
          "3D closed named round trip preserves closure and all state");
    const std::valarray<real> before3 = closed3.parameters;
    check(throws_exception<std::invalid_argument>([&closed3] {
              closed3.set_control_point(3, {2.0, 2.0, 2.0});
          }) &&
              same_values(closed3.parameters, before3),
          "3D checked mutation cannot break a closed seam");
    closed3.parameters[9] = 2.0;
    check(throws_exception<std::invalid_argument>([&closed3] {
              static_cast<void>(closed3.to_nurbs_spline());
          }) &&
              throws_exception<std::invalid_argument>([&closed3] {
                  static_cast<void>(spline3(closed3));
              }),
          "3D conversions validate a directly broken closed seam");
}

void test_interpolation2() {
    using namespace test_support;

    const std::vector<nurbspath::point2<real>> samples{
        {0.0, 0.0}, {1.0, 0.4}, {2.0, 1.2}, {3.0, 1.0}, {4.0, 0.0}};
    const std::vector<real> stations{5.0, 6.1, 7.5, 9.0, 11.0};
    auto spline = arr_spline2::interpolate(samples, stations, 3, 2e-10);
    check(spline.s_min() == stations.front() &&
              spline.s_max() == stations.back() && spline.degree() == 3 &&
              !spline.is_closed(),
          "2D array interpolation preserves native stations and metadata");
    for (std::size_t index = 0; index < samples.size(); ++index) {
        check_point2(spline.evaluate(stations[index]), samples[index], 2e-9,
                     "2D array interpolation passes through every sample");
        check(spline.get_weight(index) == 1.0,
              "2D array interpolation uses unit rational weights");
    }

    const std::vector<nurbspath::point2<real>> closed_samples{
        {1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}, {1.0, 0.0}};
    const std::vector<real> closed_stations{20.0, 21.0, 23.0, 26.0, 30.0};
    spline.adopt_to_points(closed_samples, closed_stations, 3, true, 1e-9);
    check(spline.is_closed() && spline.s_min() == 20.0 &&
              spline.s_max() == 30.0,
          "2D array adopt replaces the definition with a closed interpolant");
    for (std::size_t index = 0; index < closed_samples.size(); ++index) {
        check_point2(spline.evaluate(closed_stations[index]),
                     closed_samples[index], 3e-8,
                     "2D closed array interpolant passes through samples");
    }
}

void test_interpolation3() {
    using namespace test_support;

    const std::vector<nurbspath::point3<real>> samples{
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 0.5},
        {2.0, 4.0, 1.0},
        {3.0, 9.0, 1.5},
        {4.0, 16.0, 2.0}};
    const std::vector<real> stations{0.0, 1.2, 2.8, 5.0, 8.0};
    auto spline = arr_spline3::interpolate(samples, stations, 3, 2e-10);
    check(spline.s_min() == stations.front() &&
              spline.s_max() == stations.back() && spline.degree() == 3 &&
              !spline.is_closed(),
          "3D array interpolation preserves native stations and metadata");
    for (std::size_t index = 0; index < samples.size(); ++index) {
        check_point(spline.evaluate(stations[index]), samples[index], 2e-9,
                    "3D array interpolation passes through every sample");
        check(spline.get_weight(index) == 1.0,
              "3D array interpolation uses unit rational weights");
    }

    const std::vector<nurbspath::point3<real>> closed_samples{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.5},
        {-1.0, 0.0, 1.0},
        {0.0, -1.0, 0.5},
        {1.0, 0.0, 0.0}};
    const std::vector<real> closed_stations{2.0, 3.0, 4.0, 5.0, 6.0};
    spline.adopt_to_points(closed_samples, closed_stations, 3, true, 1e-9);
    check(spline.is_closed() && spline.s_min() == 2.0 &&
              spline.s_max() == 6.0,
          "3D array adopt replaces the definition with a closed interpolant");
    for (std::size_t index = 0; index < closed_samples.size(); ++index) {
        check_point(spline.evaluate(closed_stations[index]),
                    closed_samples[index], 3e-8,
                    "3D closed array interpolant passes through samples");
    }
}

} // namespace

int main() {
    test_standard_constructor2();
    test_standard_constructor3();
    test_flat_layout2();
    test_flat_layout3();
    test_pointer_parameters2();
    test_pointer_parameters3();
    test_validation2();
    test_validation3();
    test_evaluation_and_conversion2();
    test_evaluation_and_conversion3();
    test_curvature2();
    test_curvature3();
    test_checked_mutation2();
    test_checked_mutation3();
    test_closed_curves();
    test_interpolation2();
    test_interpolation3();
    return test_support::finish("21_test_arr_splines");
}
