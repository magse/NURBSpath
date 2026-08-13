#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <cmath>
#include <stdexcept>

namespace {

template <typename FIRST, typename SECOND>
concept has_intersection_query = requires(const FIRST& first, const SECOND& second) {
    nurbspath::intersect(first, second);
};

template <typename FIRST, typename SECOND>
concept has_distance_query = requires(const FIRST& first, const SECOND& second) {
    nurbspath::distance(first, second);
};

} // namespace

int main() {
    using namespace test_support;

    nurbspath::numerical_settings<real> settings;
    settings.residual_tolerance = 1e-9;
    settings.parameter_tolerance = 1e-11;
    settings.sample_count = 256;

    const circle2<real> unit_circle({0.0, 0.0}, 1.0);
    const auto crossing = nurbspath::intersect_ray_circle(
        ray2<real>({-2.0, 0.0}, {1.0, 0.0}), unit_circle, settings);
    check(crossing.kind == nurbspath::intersection_kind::discrete &&
              crossing.points.size() == 2,
          "2D ray-circle finds two crossings");
    check_near(crossing.points[0].s, 1.0, 1e-8, "2D intersections sorted first");
    check_near(crossing.points[1].s, 3.0, 1e-8, "2D intersections sorted second");
    check_point2(crossing.points[0].point, {-1.0, 0.0}, 1e-8,
                 "2D intersection returns point2");

    const auto tangent = nurbspath::intersect(
        ray2<real>({-2.0, 1.0}, {1.0, 0.0}), unit_circle, settings);
    check(tangent.points.size() == 1, "2D ray-circle tangent is deduplicated");
    check_near(tangent.points.front().s, 2.0, 1e-6,
               "2D tangent parameter");

    const auto endpoint = nurbspath::intersect_ray_circle(
        ray2<real>({1.0, 0.0}, {1.0, 0.0}), unit_circle, settings);
    check(endpoint.points.size() == 1 &&
              std::abs(endpoint.points.front().s) <= 1e-10,
          "2D ray endpoint contact");

    const auto miss = nurbspath::intersect_ray_circle(
        ray2<real>({-2.0, 2.0}, {1.0, 0.0}), unit_circle, settings);
    check(!miss.has_intersection(), "2D ray-circle no contact");

    const auto line = make_line2({-2.0, 0.0}, {2.0, 0.0}, 4.0, 8.0);
    const auto spline_crossing = nurbspath::intersect_spline_circle(
        line, unit_circle, settings);
    check(spline_crossing.points.size() == 2,
          "2D spline-circle finds two crossings");
    check_near(spline_crossing.points[0].s, 5.0, 1e-7,
               "2D spline first native parameter");
    check_near(spline_crossing.points[1].s, 7.0, 1e-7,
               "2D spline second native parameter");

    const auto tangent_line = make_line2({-2.0, 1.0}, {2.0, 1.0});
    check(nurbspath::intersect(tangent_line, unit_circle, settings).points.size() == 1,
          "2D spline-circle tangent contact");
    check(!nurbspath::intersect(
               make_line2({-2.0, 2.0}, {2.0, 2.0}), unit_circle, settings)
               .has_intersection(),
          "2D spline-circle no contact");

    const real root_half = std::sqrt(0.5);
    const nurbs_spline2<real> circle_arc(
        {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        2);
    check(nurbspath::intersect_spline_circle(circle_arc, unit_circle, settings)
              .is_coincident(),
          "2D coincident spline-circle arc classification");

    const nurbs_spline2<real> seam_contact(
        {{1.0, 0.0}, {2.0, 1.0}, {2.0, -1.0}, {1.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
        1,
        true,
        std::size_t(1));
    const auto seam_hits = nurbspath::intersect_spline_circle(
        seam_contact, unit_circle, settings);
    check(seam_hits.points.size() == 1 &&
              std::abs(seam_hits.points.front().s - seam_contact.s_min()) <=
                  1e-10,
          "closed 2D seam contact is returned once at the start parameter");

    const auto circle_distance = nurbspath::distance_to_circle(
        point2<real>{3.0, 0.0}, unit_circle, 1e-9);
    check_near(circle_distance.distance, 2.0, 1e-12,
               "2D point-circle distance");
    check_point2(circle_distance.closest_point, {1.0, 0.0}, 1e-12,
                 "2D point-circle closest point");
    check_near(nurbspath::distance(point2<real>{1.0 + 1e-10, 0.0},
                                   unit_circle, 1e-8),
               0.0, 0.0, "2D circle distance tolerance snapping");

    const auto spline_distance = nurbspath::distance_to_spline(
        point2<real>{0.0, 2.0},
        make_line2({-2.0, 0.0}, {2.0, 0.0}, 10.0, 14.0),
        settings);
    check_near(spline_distance.distance, 2.0, 1e-7,
               "2D point-spline distance");
    check_point2(spline_distance.closest_point, {0.0, 0.0}, 1e-6,
                 "2D point-spline closest point");
    check_near(spline_distance.s, 12.0, 1e-6,
               "2D point-spline native parameter");

    const nurbs_spline2<real> unlimited(
        {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}},
        {1.0, 1.0, 1.0},
        {-1.0, 0.0, 1.0, 2.0, 3.0},
        1,
        false,
        std::size_t(0));
    bool rejected_unlimited_intersection = false;
    try {
        static_cast<void>(
            nurbspath::intersect_spline_circle(unlimited, unit_circle, settings));
    } catch (const std::domain_error&) {
        rejected_unlimited_intersection = true;
    }
    check(rejected_unlimited_intersection,
          "2D whole-domain intersection rejects unlimited repetition");
    bool rejected_unlimited_distance = false;
    try {
        static_cast<void>(nurbspath::distance_to_spline(
            point2<real>::origin(), unlimited, settings));
    } catch (const std::domain_error&) {
        rejected_unlimited_distance = true;
    }
    check(rejected_unlimited_distance,
          "2D whole-domain distance rejects unlimited repetition");

    static_assert(has_intersection_query<ray2<real>, circle2<real>>);
    static_assert(!has_intersection_query<ray2<real>, sphere3<real>>);
    static_assert(has_distance_query<point2<real>, circle2<real>>);
    static_assert(!has_distance_query<point2<real>, sphere3<real>>);
    return finish("12_test_2d_queries");
}
