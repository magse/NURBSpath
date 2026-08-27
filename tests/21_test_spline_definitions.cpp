#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <concepts>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using real = test_support::real;
using spline2 = nurbspath::nurbs_spline2<real>;
using spline3 = nurbspath::nurbs_spline3<real>;
using definition2 = nurbspath::spline2_definition<real>;
using definition3 = nurbspath::spline3_definition<real>;

template <typename DEFINITION>
concept has_degree_field = requires(DEFINITION definition) {
    definition.degree;
};

static_assert(std::is_aggregate_v<definition2>);
static_assert(std::is_aggregate_v<definition3>);
static_assert(!has_degree_field<definition2>);
static_assert(!has_degree_field<definition3>);
static_assert(std::same_as<nurbspath::nurbs_defined_spline2<real>, spline2>);
static_assert(std::same_as<nurbspath::nurbs_defined_spline3<real>, spline3>);
static_assert(std::same_as<
              decltype(std::declval<const spline2&>().definition()),
              definition2>);
static_assert(std::same_as<
              decltype(std::declval<const spline3&>().definition()),
              definition3>);
static_assert(std::same_as<
              decltype(nurbspath::make_nurbs_defined_spline2(
                  std::declval<definition2>(), std::size_t{2})),
              std::shared_ptr<spline2>>);
static_assert(std::same_as<
              decltype(nurbspath::make_nurbs_defined_spline3(
                  std::declval<definition3>(), std::size_t{2})),
              std::shared_ptr<spline3>>);

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
    using control_vector = std::remove_cvref_t<
        decltype(std::declval<const SPLINE&>().control_points())>;
    using point_type = typename control_vector::value_type;
    using scalar_vector = std::remove_cvref_t<
        decltype(std::declval<const SPLINE&>().weights())>;
    using scalar_type = typename scalar_vector::value_type;

    explicit spline_snapshot(const SPLINE& spline)
        : control_points(spline.control_points()),
          weights(spline.weights()),
          knots(spline.knots()),
          degree(spline.degree()),
          tolerance(spline.tolerance()),
          closed(spline.is_closed()),
          start(spline.get_start()),
          end(spline.get_end()),
          sample_s((spline.s_min() + spline.s_max()) / scalar_type(2)),
          sample(spline.evaluate(sample_s)) {}

    [[nodiscard]] bool matches(const SPLINE& spline) const {
        return spline.control_points() == control_points &&
               spline.weights() == weights && spline.knots() == knots &&
               spline.degree() == degree && spline.tolerance() == tolerance &&
               spline.is_closed() == closed && spline.get_start() == start &&
               spline.get_end() == end && spline.evaluate(sample_s) == sample;
    }

    control_vector control_points;
    scalar_vector weights;
    scalar_vector knots;
    std::size_t degree;
    scalar_type tolerance;
    bool closed;
    point_type start;
    point_type end;
    scalar_type sample_s;
    point_type sample;
};

template <typename SPLINE, typename DEFINITION>
[[nodiscard]] bool matches_definition(
    const SPLINE& spline,
    const DEFINITION& definition) {
    return spline.control_points() == definition.control_points &&
           spline.weights() == definition.weights &&
           spline.knots() == definition.knots &&
           spline.is_closed() == definition.closed &&
           spline.tolerance() == definition.tolerance;
}

template <
    typename EXCEPTION = std::invalid_argument,
    typename SPLINE,
    typename DEFINITION>
void expect_invalid_update(
    SPLINE& spline,
    DEFINITION definition,
    std::string_view message) {
    const spline_snapshot<SPLINE> before(spline);
    const bool rejected = throws_exception<EXCEPTION>(
        [&spline, &definition] {
            spline.set_definition(std::move(definition));
        });
    test_support::check(rejected && before.matches(spline), message);
}

void test_definition2() {
    using namespace test_support;

    const real default_tolerance =
        real(64) * std::numeric_limits<real>::epsilon();
    definition2 draft;
    check(draft.control_points.empty() && draft.weights.empty() &&
              draft.knots.empty() && !draft.closed &&
              draft.tolerance == default_tolerance,
          "2D definition defaults form an editable empty draft");
    check(throws_exception<std::invalid_argument>([&draft] {
              static_cast<void>(spline2(draft, 2));
          }),
          "2D empty definition is rejected when adopted");

    draft.control_points = {{0.0, 0.0}, {1.0, 2.0}, {2.0, 0.0}};
    draft.weights = {1.0, 1.0, 1.0};
    draft.knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    const definition2 original = draft;

    spline2 spline(draft, 2);
    check(matches_definition(spline, original),
          "2D spline constructor clones a detached definition");
    check_point2(spline.evaluate(0.5), {1.0, 1.0}, 1e-12,
                 "2D definition constructor evaluates the expected path");

    const spline_snapshot<spline2> before_source_edit(spline);
    draft.control_points[1].y = 20.0;
    draft.weights[1] = 3.0;
    draft.knots[3] = 0.75;
    draft.closed = true;
    draft.tolerance = 1e-6;
    check(before_source_edit.matches(spline),
          "2D source definition edits do not mutate an existing spline");

    definition2 detached = spline.definition();
    const spline_snapshot<spline2> before_snapshot_edit(spline);
    detached.control_points[1].y = 9.0;
    detached.weights[1] = 2.0;
    check(before_snapshot_edit.matches(spline),
          "2D definition() returns an independent deep-copy snapshot");

    const definition2 replacement{
        {{2.0, 3.0}, {3.0, 5.0}, {1.0, 5.0}, {2.0, 3.0}},
        {1.0, 2.0, 3.0, 1.0},
        {-2.0, -2.0, -2.0, 1.0, 6.0, 6.0, 6.0},
        true,
        1e-9};
    spline.set_definition(replacement);
    check(matches_definition(spline, replacement),
          "2D definition setter commits every editable field together");
    check(spline.degree() == 2,
          "2D definition setter retains the immutable degree");
    check_point2(spline.get_start(), replacement.control_points.front(), 0.0,
                 "2D definition setter refreshes the cached start");
    check_point2(spline.get_end(), replacement.control_points.back(), 0.0,
                 "2D definition setter refreshes the cached end");
    const spline2 expected(replacement, 2);
    check_point2(spline.evaluate(2.0), expected.evaluate(2.0), 0.0,
                 "2D committed definition recalculates the path");
    check(spline.first_derivative(2.0) == expected.first_derivative(2.0) &&
              spline.second_derivative(2.0) ==
                  expected.second_derivative(2.0) &&
              spline.third_derivative(2.0) ==
                  expected.third_derivative(2.0),
          "2D committed definition refreshes analytic derivatives");

    const spline2 rvalue_constructed(
        definition2{
            original.control_points,
            original.weights,
            original.knots,
            original.closed,
            original.tolerance},
        2);
    check(matches_definition(rvalue_constructed, original),
          "2D definition constructor accepts an rvalue");

    auto made = nurbspath::make_nurbs_spline2(replacement, 2);
    auto made_defined =
        nurbspath::make_nurbs_defined_spline2(replacement, 2);
    check(made && made_defined && matches_definition(*made, replacement) &&
              matches_definition(*made_defined, replacement),
          "2D definition creators return validated shared splines");

    definition2 invalid = replacement;
    invalid.weights.pop_back();
    expect_invalid_update(
        spline, invalid, "2D mismatched weight count rolls back");
    invalid = replacement;
    invalid.knots.pop_back();
    expect_invalid_update(
        spline, invalid, "2D mismatched knot count rolls back");
    invalid = replacement;
    invalid.control_points[1].x =
        std::numeric_limits<real>::quiet_NaN();
    expect_invalid_update(
        spline, invalid, "2D nonfinite control point rolls back");
    invalid = replacement;
    invalid.weights[1] = 0.0;
    expect_invalid_update(
        spline, invalid, "2D nonpositive weight rolls back");
    invalid = replacement;
    invalid.weights[1] = std::numeric_limits<real>::infinity();
    expect_invalid_update(
        spline, invalid, "2D nonfinite weight rolls back");
    invalid = replacement;
    invalid.knots[3] = -3.0;
    expect_invalid_update(
        spline, invalid, "2D decreasing knot vector rolls back");
    invalid = replacement;
    invalid.knots[3] = std::numeric_limits<real>::quiet_NaN();
    expect_invalid_update(
        spline, invalid, "2D nonfinite knot rolls back");
    invalid = replacement;
    invalid.knots.assign(invalid.knots.size(), 0.0);
    expect_invalid_update(
        spline, invalid, "2D empty active domain rolls back");
    invalid = replacement;
    invalid.tolerance = 0.0;
    expect_invalid_update(
        spline, invalid, "2D nonpositive tolerance rolls back");
    invalid = replacement;
    invalid.tolerance = std::numeric_limits<real>::infinity();
    expect_invalid_update(
        spline, invalid, "2D nonfinite tolerance rolls back");
    invalid = replacement;
    invalid.weights.assign(invalid.weights.size(), 1e-12);
    expect_invalid_update<std::domain_error>(
        spline, invalid, "2D near-zero homogeneous weight rolls back");
    invalid = replacement;
    invalid.control_points.back() = {9.0, 9.0};
    expect_invalid_update(
        spline, invalid, "2D open closed-seam candidate rolls back");

    const definition2 degree_one_definition{
        {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}},
        {1.0, 1.0, 1.0},
        {0.0, 0.0, 0.5, 1.0, 1.0}};
    const spline2 degree_one(degree_one_definition, 1);
    check(degree_one.degree() == 1,
          "2D definition is valid with its matching degree");
    expect_invalid_update(
        spline,
        degree_one_definition,
        "2D definition incompatible with retained degree rolls back");
    check(throws_exception<std::invalid_argument>([&degree_one_definition] {
              static_cast<void>(spline2(degree_one_definition, 2));
          }),
          "2D definition constructor rejects a mismatched degree");
    check(throws_exception<std::invalid_argument>([&original] {
              static_cast<void>(spline2(original, 0));
          }),
          "2D definition constructor rejects degree zero");
    check(throws_exception<std::invalid_argument>([&original] {
              static_cast<void>(spline2(original, 3));
          }),
          "2D definition constructor rejects degree at control count");

    definition2 invalid_creator = replacement;
    invalid_creator.weights[0] = 0.0;
    check(throws_exception<std::invalid_argument>([&invalid_creator] {
              static_cast<void>(
                  nurbspath::make_nurbs_defined_spline2(invalid_creator, 2));
          }),
          "2D definition creator forwards validation failures");
}

void test_definition3() {
    using namespace test_support;

    const real default_tolerance =
        real(64) * std::numeric_limits<real>::epsilon();
    definition3 draft;
    check(draft.control_points.empty() && draft.weights.empty() &&
              draft.knots.empty() && !draft.closed &&
              draft.tolerance == default_tolerance,
          "3D definition defaults form an editable empty draft");
    check(throws_exception<std::invalid_argument>([&draft] {
              static_cast<void>(spline3(draft, 2));
          }),
          "3D empty definition is rejected when adopted");

    draft.control_points =
        {{0.0, 0.0, 0.0}, {1.0, 2.0, 3.0}, {2.0, 0.0, 0.0}};
    draft.weights = {1.0, 1.0, 1.0};
    draft.knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    const definition3 original = draft;

    spline3 spline(draft, 2);
    check(matches_definition(spline, original),
          "3D spline constructor clones a detached definition");
    check_point(spline.evaluate(0.5), {1.0, 1.0, 1.5}, 1e-12,
                "3D definition constructor evaluates the expected path");

    const spline_snapshot<spline3> before_source_edit(spline);
    draft.control_points[1].z = 30.0;
    draft.weights[1] = 3.0;
    draft.knots[3] = 0.75;
    draft.closed = true;
    draft.tolerance = 1e-6;
    check(before_source_edit.matches(spline),
          "3D source definition edits do not mutate an existing spline");

    definition3 detached = spline.definition();
    const spline_snapshot<spline3> before_snapshot_edit(spline);
    detached.control_points[1].z = 9.0;
    detached.weights[1] = 2.0;
    check(before_snapshot_edit.matches(spline),
          "3D definition() returns an independent deep-copy snapshot");

    const definition3 replacement{
        {{2.0, 3.0, 4.0},
         {3.0, 5.0, 6.0},
         {1.0, 5.0, 2.0},
         {2.0, 3.0, 4.0}},
        {1.0, 2.0, 3.0, 1.0},
        {-2.0, -2.0, -2.0, 1.0, 6.0, 6.0, 6.0},
        true,
        1e-9};
    spline.set_definition(replacement);
    check(matches_definition(spline, replacement),
          "3D definition setter commits every editable field together");
    check(spline.degree() == 2,
          "3D definition setter retains the immutable degree");
    check_point(spline.get_start(), replacement.control_points.front(), 0.0,
                "3D definition setter refreshes the cached start");
    check_point(spline.get_end(), replacement.control_points.back(), 0.0,
                "3D definition setter refreshes the cached end");
    const spline3 expected(replacement, 2);
    check_point(spline.evaluate(2.0), expected.evaluate(2.0), 0.0,
                "3D committed definition recalculates the path");
    check(spline.first_derivative(2.0) == expected.first_derivative(2.0) &&
              spline.second_derivative(2.0) ==
                  expected.second_derivative(2.0) &&
              spline.third_derivative(2.0) ==
                  expected.third_derivative(2.0),
          "3D committed definition refreshes analytic derivatives");

    const spline3 rvalue_constructed(
        definition3{
            original.control_points,
            original.weights,
            original.knots,
            original.closed,
            original.tolerance},
        2);
    check(matches_definition(rvalue_constructed, original),
          "3D definition constructor accepts an rvalue");

    auto made = nurbspath::make_nurbs_spline3(replacement, 2);
    auto made_defined =
        nurbspath::make_nurbs_defined_spline3(replacement, 2);
    check(made && made_defined && matches_definition(*made, replacement) &&
              matches_definition(*made_defined, replacement),
          "3D definition creators return validated shared splines");

    definition3 invalid = replacement;
    invalid.weights.pop_back();
    expect_invalid_update(
        spline, invalid, "3D mismatched weight count rolls back");
    invalid = replacement;
    invalid.knots.pop_back();
    expect_invalid_update(
        spline, invalid, "3D mismatched knot count rolls back");
    invalid = replacement;
    invalid.control_points[1].x =
        std::numeric_limits<real>::quiet_NaN();
    expect_invalid_update(
        spline, invalid, "3D nonfinite control point rolls back");
    invalid = replacement;
    invalid.weights[1] = 0.0;
    expect_invalid_update(
        spline, invalid, "3D nonpositive weight rolls back");
    invalid = replacement;
    invalid.weights[1] = std::numeric_limits<real>::infinity();
    expect_invalid_update(
        spline, invalid, "3D nonfinite weight rolls back");
    invalid = replacement;
    invalid.knots[3] = -3.0;
    expect_invalid_update(
        spline, invalid, "3D decreasing knot vector rolls back");
    invalid = replacement;
    invalid.knots[3] = std::numeric_limits<real>::quiet_NaN();
    expect_invalid_update(
        spline, invalid, "3D nonfinite knot rolls back");
    invalid = replacement;
    invalid.knots.assign(invalid.knots.size(), 0.0);
    expect_invalid_update(
        spline, invalid, "3D empty active domain rolls back");
    invalid = replacement;
    invalid.tolerance = 0.0;
    expect_invalid_update(
        spline, invalid, "3D nonpositive tolerance rolls back");
    invalid = replacement;
    invalid.tolerance = std::numeric_limits<real>::infinity();
    expect_invalid_update(
        spline, invalid, "3D nonfinite tolerance rolls back");
    invalid = replacement;
    invalid.weights.assign(invalid.weights.size(), 1e-12);
    expect_invalid_update<std::domain_error>(
        spline, invalid, "3D near-zero homogeneous weight rolls back");
    invalid = replacement;
    invalid.control_points.back() = {9.0, 9.0, 9.0};
    expect_invalid_update(
        spline, invalid, "3D open closed-seam candidate rolls back");

    const definition3 degree_one_definition{
        {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, {2.0, 0.0, 0.0}},
        {1.0, 1.0, 1.0},
        {0.0, 0.0, 0.5, 1.0, 1.0}};
    const spline3 degree_one(degree_one_definition, 1);
    check(degree_one.degree() == 1,
          "3D definition is valid with its matching degree");
    expect_invalid_update(
        spline,
        degree_one_definition,
        "3D definition incompatible with retained degree rolls back");
    check(throws_exception<std::invalid_argument>([&degree_one_definition] {
              static_cast<void>(spline3(degree_one_definition, 2));
          }),
          "3D definition constructor rejects a mismatched degree");
    check(throws_exception<std::invalid_argument>([&original] {
              static_cast<void>(spline3(original, 0));
          }),
          "3D definition constructor rejects degree zero");
    check(throws_exception<std::invalid_argument>([&original] {
              static_cast<void>(spline3(original, 3));
          }),
          "3D definition constructor rejects degree at control count");

    definition3 invalid_creator = replacement;
    invalid_creator.weights[0] = 0.0;
    check(throws_exception<std::invalid_argument>([&invalid_creator] {
              static_cast<void>(
                  nurbspath::make_nurbs_defined_spline3(invalid_creator, 2));
          }),
          "3D definition creator forwards validation failures");
}

} // namespace

int main() {
    test_definition2();
    test_definition3();
    return test_support::finish("21_test_spline_definitions");
}
