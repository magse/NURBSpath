#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <cmath>

int main() {
    using namespace test_support;

    nurbspath::numerical_settings<real> settings;
    settings.residual_tolerance = 1e-9;
    settings.parameter_tolerance = 1e-12;
    settings.sample_count = 512;

    const auto line = make_line({-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    const sphere3<real> sphere({0.0, 0.0, 0.0}, 1.0);
    const auto sphere_hits = nurbspath::intersect_spline_sphere(line, sphere, settings);
    check(sphere_hits.points.size() == 2, "spline-sphere has two intersections");
    const auto endpoint_line = make_line({1.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    const auto endpoint_hits = nurbspath::intersect_spline_sphere(
        endpoint_line, sphere, settings);
    check(endpoint_hits.points.size() == 1, "endpoint contact is retained");

    const auto missing_line = make_line({-2.0, 2.0, 0.0}, {2.0, 2.0, 0.0});
    check(!nurbspath::intersect_spline_sphere(missing_line, sphere, settings)
               .has_intersection(),
          "spline-sphere no contact");

    const auto vertical = make_line({0.0, 0.0, -1.0}, {0.0, 0.0, 1.0});
    const plane3<real> plane({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    check(nurbspath::intersect_spline_plane(vertical, plane, settings)
              .points.size() == 1,
          "spline-plane has one intersection");

    const auto tangent_line = make_line({-2.0, 1.0, 0.0}, {2.0, 1.0, 0.0});
    check(nurbspath::intersect_spline_sphere(tangent_line, sphere, settings)
              .points.size() == 1,
          "tangent contact is retained");

    const sphere3<real> large_sphere({0.0, 0.0, 0.0}, 1.0e6);
    const auto flat_tangent = make_line(
        {-2.0, 1.0e6, 0.0}, {2.0, 1.0e6, 0.0});
    check(nurbspath::intersect_spline_sphere(flat_tangent, large_sphere, settings)
              .points.size() == 1,
          "flat tangent band is deduplicated");
    check(nurbspath::intersect_spline_plane(tangent_line, plane, settings)
              .is_coincident(),
          "coplanar spline is coincident");

    const real root_half = std::sqrt(0.5);
    const nurbs_spline3<real> circle_arc(
        {{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0}},
        {1.0, root_half, 1.0},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
        2);
    check(nurbspath::intersect_spline_sphere(circle_arc, sphere, settings)
              .is_coincident(),
          "spline on sphere is coincident");

    const nurbs_spline3<real> seam_contact(
        {{0.0, 0.0, 0.0},
         {1.0, 0.0, 1.0},
         {-1.0, 0.0, 1.0},
         {0.0, 0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
        1,
        true);
    const auto seam_hits = nurbspath::intersect_spline_plane(
        seam_contact, plane, settings);
    check(seam_hits.points.size() == 1 &&
              std::abs(seam_hits.points.front().s - seam_contact.s_min()) <=
                  1e-10,
          "closed spline-plane seam contact is returned once");

    return finish("06_test_spline_intersections");
}
