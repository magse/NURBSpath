#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <string>
#include <type_traits>

int main() {
    using namespace test_support;
    using float_point = point3<float>;

    const nurbspath::nurbs_spline3<float> line(
        {float_point{0.0F, 0.0F, -1.0F}, float_point{0.0F, 0.0F, 1.0F}},
        {1.0F, 1.0F},
        {0.0F, 0.0F, 1.0F, 1.0F},
        1);
    const nurbspath::plane3<float> plane(
        float_point{0.0F, 0.0F, 0.0F}, vector3<float>::unit_z());
    nurbspath::numerical_settings<float> settings;
    settings.residual_tolerance = 1e-5F;
    settings.parameter_tolerance = 1e-6F;
    settings.sample_count = 64;
    check(nurbspath::intersect_spline_plane(line, plane, settings).points.size() == 1,
          "float geometry template");

    const auto periodic = nurbspath::nurbs_spline3<float>::interpolate(
        {float_point{1.0F, 0.0F, 0.0F},
         float_point{0.0F, 1.0F, 0.0F},
         float_point{-1.0F, 0.0F, 0.0F},
         float_point{0.0F, -1.0F, 0.0F},
         float_point{1.0F, 0.0F, 0.0F}},
        {0.0F, 1.0F, 2.0F, 3.0F, 4.0F},
        3,
        true,
        std::size_t(2),
        1e-5F);
    check(periodic.is_periodic() &&
              periodic.evaluate(4.5F).approximately_equal(
                  periodic.evaluate(0.5F), 1e-4F),
          "float periodic spline advances to its next period");

    const auto view = nurbspath::svg_view3<float>::orthographic(
        float_point{3.0F, -4.0F, 2.0F},
        float_point::origin(),
        6.0F,
        4.0F,
        600,
        400);
    check(nurbspath::to_svg(view, line).find("nurbspath_spline") !=
              std::string::npos,
          "float SVG template");

    static_assert(std::is_aggregate_v<
                  nurbspath::detail::svg_camera_point3<double>>);
    static_assert(std::is_aggregate_v<
                  nurbspath::detail::svg_screen_point3<double>>);
    check(nurbspath::detail::kind_for_count<double>(0) ==
              nurbspath::intersection_kind::none,
          "implementation intersection helper is in detail namespace");
    return finish("09_test_float_and_detail");
}
