#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <concepts>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

template <typename SPLINE>
concept has_degree_setter = requires(SPLINE& spline) {
    spline.set_degree(std::size_t{1});
};

template <typename SPLINE>
concept has_start_setter = requires(SPLINE& spline) {
    spline.set_start(spline.get_start());
};

template <typename SPLINE>
concept has_end_setter = requires(SPLINE& spline) {
    spline.set_end(spline.get_end());
};

static_assert(!has_degree_setter<test_support::nurbs_spline2<test_support::real>>);
static_assert(!has_degree_setter<test_support::nurbs_spline3<test_support::real>>);
static_assert(
    !has_start_setter<test_support::nurbs_spline3<test_support::real>>);
static_assert(!has_end_setter<test_support::nurbs_spline3<test_support::real>>);
using spline3_type = test_support::nurbs_spline3<test_support::real>;
static_assert(std::is_same_v<
              decltype(std::declval<const spline3_type&>().get_start()),
              const test_support::point3<test_support::real>&>);
static_assert(std::is_same_v<
              decltype(std::declval<const spline3_type&>().get_end()),
              const test_support::point3<test_support::real>&>);

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
        decltype(std::declval<const SPLINE&>().get_control_points())>;
    using point_type = typename control_vector::value_type;
    using weight_vector = std::remove_cvref_t<
        decltype(std::declval<const SPLINE&>().get_weights())>;
    using real_type = typename weight_vector::value_type;

    explicit spline_snapshot(const SPLINE& spline)
        : control_points(spline.get_control_points()),
          weights(spline.get_weights()),
          knots(spline.get_knots()),
          degree(spline.degree()),
          tolerance(spline.tolerance()),
          closed(spline.is_closed()),
          start(spline.get_start()),
          end(spline.get_end()),
          sample_s((spline.s_min() + spline.s_max()) / real_type(2)),
          sample(spline.evaluate(sample_s)) {}

    [[nodiscard]] bool matches(const SPLINE& spline) const {
        return spline.get_control_points() == control_points &&
               spline.get_weights() == weights &&
               spline.get_knots() == knots &&
               spline.degree() == degree && spline.tolerance() == tolerance &&
               spline.is_closed() == closed && spline.get_start() == start &&
               spline.get_end() == end && spline.evaluate(sample_s) == sample;
    }

    control_vector control_points;
    weight_vector weights;
    weight_vector knots;
    std::size_t degree;
    real_type tolerance;
    bool closed;
    point_type start;
    point_type end;
    real_type sample_s;
    point_type sample;
};

void test_spline2_mutation() {
    using namespace test_support;

    const std::vector<point2<real>> controls{
        {0.0, 0.0}, {1.0, 2.0}, {2.0, 0.0}};
    const std::vector<real> weights{1.0, 1.0, 1.0};
    const std::vector<real> knots{0.0, 0.0, 0.0, 1.0, 1.0, 1.0};

    nurbs_spline2<real> control_curve(controls, weights, knots, 2);
    check_point2(control_curve.get_control_point(1), controls[1], 0.0,
                 "2D indexed control-point getter");
    check_near(control_curve.get_weight(1), 1.0, 0.0,
               "2D indexed weight getter");
    check_near(control_curve.get_knot(3), 1.0, 0.0,
               "2D indexed knot getter");

    point2<real> changed_inner = control_curve.get_control_point(1);
    changed_inner.y = 4.0;
    control_curve.set_control_point(1, changed_inner);
    check_point2(control_curve.evaluate(0.5), {1.0, 2.0}, 1e-12,
                 "2D inner control coordinate changes the curve");
    control_curve.set_control_point(0, {-1.0, 0.0});
    check_point2(control_curve.get_start(), {-1.0, 0.0}, 0.0,
                 "2D endpoint control mutation refreshes cached start");
    check_point2(control_curve.get_start(),
                 control_curve.evaluate(control_curve.s_min()), 0.0,
                 "2D cached start matches evaluation after control mutation");

    nurbs_spline2<real> weight_curve(controls, weights, knots, 2);
    weight_curve.set_weight(1, 2.0);
    check_point2(weight_curve.evaluate(0.5), {1.0, 4.0 / 3.0}, 1e-12,
                 "2D inner weight changes rational evaluation");
    weight_curve.set_weights({1.0, 3.0, 1.0});
    check_point2(weight_curve.evaluate(0.5), {1.0, 1.5}, 1e-12,
                 "2D complete weight replacement is applied");

    nurbs_spline2<real> rescaled_curve(controls, weights, knots, 2);
    rescaled_curve.set_knots({10.0, 10.0, 10.0, 14.0, 14.0, 14.0});
    check_near(rescaled_curve.s_min(), 10.0, 0.0,
               "2D knot replacement changes lower domain bound");
    check_near(rescaled_curve.s_max(), 14.0, 0.0,
               "2D knot replacement changes upper domain bound");
    check_point2(rescaled_curve.evaluate(12.0), {1.0, 1.0}, 1e-12,
                 "2D affine knot rescaling preserves curve geometry");
    check(rescaled_curve.first_derivative(12.0).approximately_equal(
              {0.5, 0.0}, 1e-12),
          "2D affine knot rescaling adjusts native-s derivative");

    nurbs_spline2<real> standard_curve(controls, weights, knots, 2);
    standard_curve.set_standard_knots(4.0);
    check(standard_curve.get_knots() ==
              std::vector<real>{0.0, 0.0, 0.0, 4.0, 4.0, 4.0},
          "2D one-argument standard knots use zero and the requested end");
    check_near(standard_curve.s_min(), 0.0, 0.0,
               "2D one-argument standard knots start at zero");
    check_near(standard_curve.s_max(), 4.0, 0.0,
               "2D one-argument standard knots use the requested end");
    check_point2(standard_curve.get_start(), standard_curve.evaluate(0.0),
                 0.0, "2D standard knots refresh cached start");
    check_point2(standard_curve.get_end(), standard_curve.evaluate(4.0),
                 0.0, "2D standard knots refresh cached end");

    standard_curve.set_standard_knots(-2.0, 6.0);
    check(standard_curve.get_knots() ==
              std::vector<real>{-2.0, -2.0, -2.0, 6.0, 6.0, 6.0},
          "2D two-argument standard knots use the requested domain");
    check_near(standard_curve.s_min(), -2.0, 0.0,
               "2D two-argument standard knots change the lower bound");
    check_near(standard_curve.s_max(), 6.0, 0.0,
               "2D two-argument standard knots change the upper bound");
    check_point2(standard_curve.get_start(),
                 standard_curve.evaluate(standard_curve.s_min()), 0.0,
                 "2D two-argument standard knots keep the start cache current");
    check_point2(standard_curve.get_end(),
                 standard_curve.evaluate(standard_curve.s_max()), 0.0,
                 "2D two-argument standard knots keep the end cache current");

    const std::vector<point2<real>> cache_controls{
        {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
    const std::vector<real> cache_weights(4, 1.0);
    const std::vector<real> cache_knots{0.0, 1.0, 2.0, 3.0,
                                        4.0, 5.0, 6.0};
    nurbs_spline2<real> weight_cache_curve(
        cache_controls, cache_weights, cache_knots, 2);
    check_point2(weight_cache_curve.get_start(), {1.0, 0.0}, 1e-12,
                 "2D nonclamped fixture has expected cached start");
    weight_cache_curve.set_weight(0, 3.0);
    check_point2(weight_cache_curve.get_start(), {0.5, 0.0}, 1e-12,
                 "2D endpoint-affecting weight refreshes cached start");
    check_point2(weight_cache_curve.get_start(),
                 weight_cache_curve.evaluate(weight_cache_curve.s_min()),
                 0.0, "2D weight mutation cache agrees with evaluation");

    nurbs_spline2<real> knot_cache_curve(
        cache_controls, cache_weights, cache_knots, 2);
    knot_cache_curve.set_knot(1, 0.0);
    check_near(knot_cache_curve.get_knot(1), 0.0, 0.0,
               "2D indexed knot setter updates selected knot");
    check_point2(knot_cache_curve.get_start(), {4.0 / 3.0, 0.0}, 1e-12,
                 "2D endpoint-affecting knot refreshes cached start");
    check_point2(knot_cache_curve.get_start(),
                 knot_cache_curve.evaluate(knot_cache_curve.s_min()), 0.0,
                 "2D knot mutation cache agrees with evaluation");

    nurbs_spline2<real> redefined(controls, weights, knots, 2);
    const std::vector<point2<real>> replacement_controls{
        {0.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {3.0, 0.0}};
    const std::vector<real> replacement_weights{1.0, 2.0, 2.0, 1.0};
    const std::vector<real> replacement_knots{
        0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0};
    redefined.set_definition(
        replacement_controls,
        replacement_weights,
        replacement_knots,
        false,
        1e-9);
    check(redefined.get_control_points() == replacement_controls &&
              redefined.get_weights() == replacement_weights &&
              redefined.get_knots() == replacement_knots,
          "2D atomic definition replacement changes coupled vectors");
    check(redefined.degree() == 2,
          "2D atomic definition replacement retains degree");
    check_near(redefined.tolerance(), 1e-9, 0.0,
               "2D atomic definition replacement changes tolerance");
    check_point2(redefined.get_start(), replacement_controls.front(), 0.0,
                 "2D definition replacement refreshes cached start");
    check_point2(redefined.get_end(), replacement_controls.back(), 0.0,
                 "2D definition replacement refreshes cached end");

    nurbs_spline2<real> closed_curve(
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
        1,
        true);
    const spline_snapshot closed_before(closed_curve);
    check(throws_exception<std::invalid_argument>([&closed_curve] {
              closed_curve.set_control_point(0, {0.25, 0.0});
          }) && closed_before.matches(closed_curve),
          "2D seam-breaking point mutation rolls back completely");
    auto moved_closed_controls = closed_curve.get_control_points();
    moved_closed_controls.front() = {2.0, 2.0};
    moved_closed_controls.back() = {2.0, 2.0};
    closed_curve.set_control_points(moved_closed_controls);
    check(closed_curve.is_closed(),
          "2D atomic seam movement preserves closure state");
    check_point2(closed_curve.get_start(), {2.0, 2.0}, 0.0,
                 "2D atomic seam movement refreshes cached start");
    check_point2(closed_curve.get_end(), {2.0, 2.0}, 0.0,
                 "2D atomic seam movement refreshes cached end");
    closed_curve.set_closed(false);
    closed_curve.set_control_point(0, {3.0, 2.0});
    const spline_snapshot broken_open(closed_curve);
    check(throws_exception<std::invalid_argument>([&closed_curve] {
              closed_curve.set_closed(true);
          }) && broken_open.matches(closed_curve),
          "2D invalid closure request rolls back completely");
    closed_curve.set_control_point(
        0, closed_curve.get_control_points().back());
    closed_curve.set_closed(true);
    check(closed_curve.is_closed(),
          "2D closure can be restored after repairing the seam");

    nurbs_spline2<real> near_closed(
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1e-3}},
        {1.0, 1.0, 1.0},
        {0.0, 0.0, 0.5, 1.0, 1.0},
        1,
        true,
        1e-4);
    const spline_snapshot tolerance_before(near_closed);
    check(throws_exception<std::invalid_argument>([&near_closed] {
              near_closed.set_tolerance(1e-5);
          }) && tolerance_before.matches(near_closed),
          "2D seam-invalidating tolerance reduction rolls back");
    near_closed.set_tolerance(2e-4);
    check_near(near_closed.tolerance(), 2e-4, 0.0,
               "2D valid tolerance update is applied");

    nurbs_spline2<real> invalid_curve(controls, weights, knots, 2);
    const spline_snapshot invalid_before(invalid_curve);
    const real nan = std::numeric_limits<real>::quiet_NaN();
    const auto unchanged_after = [&](bool rejected, std::string_view message) {
        check(rejected && invalid_before.matches(invalid_curve), message);
    };
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            static_cast<void>(invalid_curve.get_control_point(99));
        }),
        "2D out-of-range control getter leaves definition unchanged");
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            static_cast<void>(invalid_curve.get_weight(99));
        }),
        "2D out-of-range weight getter leaves definition unchanged");
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            static_cast<void>(invalid_curve.get_knot(99));
        }),
        "2D out-of-range knot getter leaves definition unchanged");
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            invalid_curve.set_control_point(99, {0.0, 0.0});
        }),
        "2D out-of-range control setter rolls back");
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            invalid_curve.set_weight(99, 1.0);
        }),
        "2D out-of-range weight setter rolls back");
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            invalid_curve.set_knot(99, 0.0);
        }),
        "2D out-of-range knot setter rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve, nan] {
            invalid_curve.set_control_point(1, {nan, 0.0});
        }),
        "2D nonfinite control mutation rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_weight(1, 0.0);
        }),
        "2D nonpositive weight mutation rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_knot(3, -1.0);
        }),
        "2D decreasing knot mutation rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_control_points({{0.0, 0.0}, {1.0, 0.0}});
        }),
        "2D wrong-sized control replacement rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_weights({1.0, 1.0});
        }),
        "2D wrong-sized weight replacement rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_knots({0.0, 0.0, 0.0, 1.0, 1.0});
        }),
        "2D wrong-sized knot replacement rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_standard_knots(0.0);
        }),
        "2D zero standard-knot domain rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve, nan] {
            invalid_curve.set_standard_knots(nan);
        }),
        "2D nonfinite standard-knot end rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_standard_knots(2.0, 2.0);
        }),
        "2D empty two-bound standard-knot domain rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_tolerance(0.0);
        }),
        "2D invalid tolerance mutation rolls back");
    auto invalid_weights = invalid_curve.get_weights();
    invalid_weights[1] = 0.0;
    unchanged_after(
        throws_exception<std::invalid_argument>([&] {
            invalid_curve.set_definition(
                invalid_curve.get_control_points(),
                invalid_weights,
                invalid_curve.get_knots(),
                false,
                invalid_curve.tolerance());
        }),
        "2D invalid atomic definition replacement rolls back");
}

void test_spline3_mutation() {
    using namespace test_support;

    const std::vector<point3<real>> controls{
        {0.0, 0.0, 0.0}, {1.0, 2.0, 3.0}, {2.0, 0.0, 0.0}};
    const std::vector<real> weights{1.0, 1.0, 1.0};
    const std::vector<real> knots{0.0, 0.0, 0.0, 1.0, 1.0, 1.0};

    nurbs_spline3<real> control_curve(controls, weights, knots, 2);
    check_point(control_curve.get_control_point(1), controls[1], 0.0,
                "3D indexed control-point getter");
    check_near(control_curve.get_weight(1), 1.0, 0.0,
               "3D indexed weight getter");
    check_near(control_curve.get_knot(3), 1.0, 0.0,
               "3D indexed knot getter");

    point3<real> changed_inner = control_curve.get_control_point(1);
    changed_inner.z = 5.0;
    control_curve.set_control_point(1, changed_inner);
    check_point(control_curve.evaluate(0.5), {1.0, 1.0, 2.5}, 1e-12,
                "3D inner control coordinate changes the curve");
    control_curve.set_control_point(0, {-1.0, 0.0, 0.0});
    check_point(control_curve.get_start(), {-1.0, 0.0, 0.0}, 0.0,
                "3D endpoint control mutation refreshes cached start");
    check_point(control_curve.get_start(),
                control_curve.evaluate(control_curve.s_min()), 0.0,
                "3D cached start matches evaluation after control mutation");
    auto endpoint_controls = control_curve.get_control_points();
    endpoint_controls.front() = {-2.0, 1.0, 0.5};
    endpoint_controls.back() = {3.0, -1.0, 0.25};
    control_curve.set_control_points(endpoint_controls);
    check_point(control_curve.get_start(), endpoint_controls.front(), 0.0,
                "3D clamped start is placed through the control vector");
    check_point(control_curve.get_end(), endpoint_controls.back(), 0.0,
                "3D clamped end is placed through the control vector");
    check_point(control_curve.get_start(),
                control_curve.evaluate(control_curve.s_min()), 0.0,
                "3D start getter matches active-domain evaluation");
    check_point(control_curve.get_end(),
                control_curve.evaluate(control_curve.s_max()), 0.0,
                "3D end getter matches active-domain evaluation");

    nurbs_spline3<real> weight_curve(controls, weights, knots, 2);
    weight_curve.set_weight(1, 2.0);
    check_point(weight_curve.evaluate(0.5), {1.0, 4.0 / 3.0, 2.0}, 1e-12,
                "3D inner weight changes rational evaluation");
    weight_curve.set_weights({1.0, 3.0, 1.0});
    check_point(weight_curve.evaluate(0.5), {1.0, 1.5, 2.25}, 1e-12,
                "3D complete weight replacement is applied");

    nurbs_spline3<real> rescaled_curve(controls, weights, knots, 2);
    rescaled_curve.set_knots({10.0, 10.0, 10.0, 14.0, 14.0, 14.0});
    check_near(rescaled_curve.s_min(), 10.0, 0.0,
               "3D knot replacement changes lower domain bound");
    check_near(rescaled_curve.s_max(), 14.0, 0.0,
               "3D knot replacement changes upper domain bound");
    check_point(rescaled_curve.evaluate(12.0), {1.0, 1.0, 1.5}, 1e-12,
                "3D affine knot rescaling preserves curve geometry");
    check(rescaled_curve.first_derivative(12.0).approximately_equal(
              {0.5, 0.0, 0.0}, 1e-12),
          "3D affine knot rescaling adjusts native-s derivative");

    nurbs_spline3<real> standard_curve(controls, weights, knots, 2);
    standard_curve.set_standard_knots(4.0);
    check(standard_curve.get_knots() ==
              std::vector<real>{0.0, 0.0, 0.0, 4.0, 4.0, 4.0},
          "3D one-argument standard knots use zero and the requested end");
    check(standard_curve.degree() == 2,
          "3D standard knots retain degree");
    check_near(standard_curve.s_min(), 0.0, 0.0,
               "3D one-argument standard knots start at zero");
    check_near(standard_curve.s_max(), 4.0, 0.0,
               "3D one-argument standard knots use the requested end");
    check_point(standard_curve.evaluate(2.0), {1.0, 1.0, 1.5}, 1e-12,
                "3D standard knots preserve single-span curve geometry");
    check(standard_curve.first_derivative(2.0).approximately_equal(
              {0.5, 0.0, 0.0}, 1e-12),
          "3D standard knots apply native-domain derivative scaling");
    check_point(standard_curve.get_start(), standard_curve.evaluate(0.0),
                0.0, "3D standard knots refresh cached start");
    check_point(standard_curve.get_end(), standard_curve.evaluate(4.0), 0.0,
                "3D standard knots refresh cached end");

    standard_curve.set_standard_knots(-2.0, 6.0);
    check(standard_curve.get_knots() ==
              std::vector<real>{-2.0, -2.0, -2.0, 6.0, 6.0, 6.0},
          "3D two-argument standard knots use the requested domain");
    check_near(standard_curve.s_min(), -2.0, 0.0,
               "3D two-argument standard knots change the lower bound");
    check_near(standard_curve.s_max(), 6.0, 0.0,
               "3D two-argument standard knots change the upper bound");
    check_point(standard_curve.evaluate(2.0), {1.0, 1.0, 1.5}, 1e-12,
                "3D two-argument standard knots preserve midpoint geometry");
    check(standard_curve.first_derivative(2.0).approximately_equal(
              {0.25, 0.0, 0.0}, 1e-12),
          "3D two-argument standard knots scale derivatives by domain width");

    nurbs_spline3<real> multi_span_standard(
        {{0.0, 0.0, 0.0},
         {1.0, 2.0, 0.0},
         {2.0, 1.0, 1.0},
         {3.0, 2.0, 0.0},
         {4.0, 0.0, 0.0}},
        std::vector<real>(5, 1.0),
        {0.0, 0.0, 0.0, 0.25, 0.75, 1.0, 1.0, 1.0},
        2);
    multi_span_standard.set_standard_knots(-3.0, 3.0);
    check(multi_span_standard.get_knots().size() == 8 &&
              multi_span_standard.get_knot(0) == -3.0 &&
              multi_span_standard.get_knot(1) == -3.0 &&
              multi_span_standard.get_knot(2) == -3.0 &&
              multi_span_standard.get_knot(5) == 3.0 &&
              multi_span_standard.get_knot(6) == 3.0 &&
              multi_span_standard.get_knot(7) == 3.0,
          "3D multi-span standard knots clamp both endpoint blocks");
    check_near(multi_span_standard.get_knot(3), -1.0, 1e-12,
               "3D first standard interior knot is uniformly spaced");
    check_near(multi_span_standard.get_knot(4), 1.0, 1e-12,
               "3D second standard interior knot is uniformly spaced");
    check_near(multi_span_standard.s_min(), -3.0, 0.0,
               "3D multi-span standard knots use the requested start");
    check_near(multi_span_standard.s_max(), 3.0, 0.0,
               "3D multi-span standard knots use the requested end");
    check_point(multi_span_standard.get_start(), {0.0, 0.0, 0.0}, 0.0,
                "3D multi-span standard knots clamp the start");
    check_point(multi_span_standard.get_end(), {4.0, 0.0, 0.0}, 0.0,
                "3D multi-span standard knots clamp the end");
    check_point(multi_span_standard.get_start(),
                multi_span_standard.evaluate(multi_span_standard.s_min()),
                0.0, "3D multi-span standard-knot start cache is current");
    check_point(multi_span_standard.get_end(),
                multi_span_standard.evaluate(multi_span_standard.s_max()),
                0.0, "3D multi-span standard-knot end cache is current");

    const spline_snapshot narrow_domain_before(multi_span_standard);
    check(throws_exception<std::invalid_argument>([&multi_span_standard] {
              multi_span_standard.set_standard_knots(
                  1.0, 1.0 + std::numeric_limits<real>::epsilon());
          }) && narrow_domain_before.matches(multi_span_standard),
          "3D unrepresentable standard interior spacing rolls back");

    const std::vector<point3<real>> cache_controls{
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 2.0},
        {2.0, 2.0, 2.0},
        {0.0, 2.0, 0.0}};
    const std::vector<real> cache_weights(4, 1.0);
    const std::vector<real> cache_knots{0.0, 1.0, 2.0, 3.0,
                                        4.0, 5.0, 6.0};
    nurbs_spline3<real> weight_cache_curve(
        cache_controls, cache_weights, cache_knots, 2);
    check_point(weight_cache_curve.get_start(), {1.0, 0.0, 1.0}, 1e-12,
                "3D nonclamped fixture has expected cached start");
    weight_cache_curve.set_weight(0, 3.0);
    check_point(weight_cache_curve.get_start(), {0.5, 0.0, 0.5}, 1e-12,
                "3D endpoint-affecting weight refreshes cached start");
    check_point(weight_cache_curve.get_start(),
                weight_cache_curve.evaluate(weight_cache_curve.s_min()),
                0.0, "3D weight mutation cache agrees with evaluation");

    nurbs_spline3<real> knot_cache_curve(
        cache_controls, cache_weights, cache_knots, 2);
    knot_cache_curve.set_knot(1, 0.0);
    check_near(knot_cache_curve.get_knot(1), 0.0, 0.0,
               "3D indexed knot setter updates selected knot");
    check_point(knot_cache_curve.get_start(),
                {4.0 / 3.0, 0.0, 4.0 / 3.0}, 1e-12,
                "3D endpoint-affecting knot refreshes cached start");
    check_point(knot_cache_curve.get_start(),
                knot_cache_curve.evaluate(knot_cache_curve.s_min()), 0.0,
                "3D knot mutation cache agrees with evaluation");

    nurbs_spline3<real> redefined(controls, weights, knots, 2);
    const std::vector<point3<real>> replacement_controls{
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0},
        {2.0, 1.0, 1.0},
        {3.0, 0.0, 0.0}};
    const std::vector<real> replacement_weights{1.0, 2.0, 2.0, 1.0};
    const std::vector<real> replacement_knots{
        0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0};
    redefined.set_definition(
        replacement_controls,
        replacement_weights,
        replacement_knots,
        false,
        1e-9);
    check(redefined.get_control_points() == replacement_controls &&
              redefined.get_weights() == replacement_weights &&
              redefined.get_knots() == replacement_knots,
          "3D atomic definition replacement changes coupled vectors");
    check(redefined.degree() == 2,
          "3D atomic definition replacement retains degree");
    check_near(redefined.tolerance(), 1e-9, 0.0,
               "3D atomic definition replacement changes tolerance");
    check_point(redefined.get_start(), replacement_controls.front(), 0.0,
                "3D definition replacement refreshes cached start");
    check_point(redefined.get_end(), replacement_controls.back(), 0.0,
                "3D definition replacement refreshes cached end");

    nurbs_spline3<real> closed_curve(
        {{0.0, 0.0, 0.0},
         {1.0, 0.0, 0.0},
         {0.0, 1.0, 0.0},
         {0.0, 0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
        1,
        true);
    const spline_snapshot closed_before(closed_curve);
    check(throws_exception<std::invalid_argument>([&closed_curve] {
              closed_curve.set_control_point(0, {0.25, 0.0, 0.0});
          }) && closed_before.matches(closed_curve),
          "3D seam-breaking point mutation rolls back completely");
    auto moved_closed_controls = closed_curve.get_control_points();
    moved_closed_controls.front() = {2.0, 2.0, 2.0};
    moved_closed_controls.back() = {2.0, 2.0, 2.0};
    closed_curve.set_control_points(moved_closed_controls);
    check(closed_curve.is_closed(),
          "3D atomic seam movement preserves closure state");
    check_point(closed_curve.get_start(), {2.0, 2.0, 2.0}, 0.0,
                "3D atomic seam movement refreshes cached start");
    check_point(closed_curve.get_end(), {2.0, 2.0, 2.0}, 0.0,
                "3D atomic seam movement refreshes cached end");
    closed_curve.set_closed(false);
    closed_curve.set_control_point(0, {3.0, 2.0, 2.0});
    const spline_snapshot broken_open(closed_curve);
    check(throws_exception<std::invalid_argument>([&closed_curve] {
              closed_curve.set_closed(true);
          }) && broken_open.matches(closed_curve),
          "3D invalid closure request rolls back completely");
    closed_curve.set_control_point(
        0, closed_curve.get_control_points().back());
    closed_curve.set_closed(true);
    check(closed_curve.is_closed(),
          "3D closure can be restored after repairing the seam");

    nurbs_spline3<real> near_closed(
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1e-3}},
        {1.0, 1.0, 1.0},
        {0.0, 0.0, 0.5, 1.0, 1.0},
        1,
        true,
        1e-4);
    const spline_snapshot tolerance_before(near_closed);
    check(throws_exception<std::invalid_argument>([&near_closed] {
              near_closed.set_tolerance(1e-5);
          }) && tolerance_before.matches(near_closed),
          "3D seam-invalidating tolerance reduction rolls back");
    near_closed.set_tolerance(2e-4);
    check_near(near_closed.tolerance(), 2e-4, 0.0,
               "3D valid tolerance update is applied");

    nurbs_spline3<real> invalid_curve(controls, weights, knots, 2);
    const spline_snapshot invalid_before(invalid_curve);
    const real nan = std::numeric_limits<real>::quiet_NaN();
    const auto unchanged_after = [&](bool rejected, std::string_view message) {
        check(rejected && invalid_before.matches(invalid_curve), message);
    };
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            static_cast<void>(invalid_curve.get_control_point(99));
        }),
        "3D out-of-range control getter leaves definition unchanged");
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            static_cast<void>(invalid_curve.get_weight(99));
        }),
        "3D out-of-range weight getter leaves definition unchanged");
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            static_cast<void>(invalid_curve.get_knot(99));
        }),
        "3D out-of-range knot getter leaves definition unchanged");
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            invalid_curve.set_control_point(99, {0.0, 0.0, 0.0});
        }),
        "3D out-of-range control setter rolls back");
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            invalid_curve.set_weight(99, 1.0);
        }),
        "3D out-of-range weight setter rolls back");
    unchanged_after(
        throws_exception<std::out_of_range>([&invalid_curve] {
            invalid_curve.set_knot(99, 0.0);
        }),
        "3D out-of-range knot setter rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve, nan] {
            invalid_curve.set_control_point(1, {nan, 0.0, 0.0});
        }),
        "3D nonfinite control mutation rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_weight(1, 0.0);
        }),
        "3D nonpositive weight mutation rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_knot(3, -1.0);
        }),
        "3D decreasing knot mutation rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_control_points(
                {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});
        }),
        "3D wrong-sized control replacement rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_weights({1.0, 1.0});
        }),
        "3D wrong-sized weight replacement rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_knots({0.0, 0.0, 0.0, 1.0, 1.0});
        }),
        "3D wrong-sized knot replacement rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_standard_knots(0.0);
        }),
        "3D zero standard-knot domain rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_standard_knots(-1.0);
        }),
        "3D reversed zero-based standard-knot domain rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve, nan] {
            invalid_curve.set_standard_knots(nan);
        }),
        "3D nonfinite standard-knot end rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_standard_knots(
                std::numeric_limits<real>::denorm_min());
        }),
        "3D numerically unusable standard-knot span rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_standard_knots(2.0, 2.0);
        }),
        "3D empty two-bound standard-knot domain rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_standard_knots(3.0, 2.0);
        }),
        "3D reversed two-bound standard-knot domain rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_standard_knots(
                0.0, std::numeric_limits<real>::infinity());
        }),
        "3D infinite standard-knot bound rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_standard_knots(
                -std::numeric_limits<real>::max(),
                std::numeric_limits<real>::max());
        }),
        "3D overflowing standard-knot domain width rolls back");
    unchanged_after(
        throws_exception<std::invalid_argument>([&invalid_curve] {
            invalid_curve.set_tolerance(0.0);
        }),
        "3D invalid tolerance mutation rolls back");
    auto invalid_weights = invalid_curve.get_weights();
    invalid_weights[1] = 0.0;
    unchanged_after(
        throws_exception<std::invalid_argument>([&] {
            invalid_curve.set_definition(
                invalid_curve.get_control_points(),
                invalid_weights,
                invalid_curve.get_knots(),
                false,
                invalid_curve.tolerance());
        }),
        "3D invalid atomic definition replacement rolls back");
}

} // namespace

int main() {
    test_spline2_mutation();
    test_spline3_mutation();
    return test_support::finish("19_test_spline_mutation");
}
