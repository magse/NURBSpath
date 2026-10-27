#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <limits>
#include <stdexcept>
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

    const auto closed = nurbspath::nurbs_spline3<float>::interpolate(
        {float_point{1.0F, 0.0F, 0.0F},
         float_point{0.0F, 1.0F, 0.0F},
         float_point{-1.0F, 0.0F, 0.0F},
         float_point{0.0F, -1.0F, 0.0F},
         float_point{1.0F, 0.0F, 0.0F}},
        {0.0F, 1.0F, 2.0F, 3.0F, 4.0F},
        3,
        true,
        1e-5F);
    check(closed.is_closed() &&
              closed.get_start().approximately_equal(closed.get_end(), 1e-4F),
          "float closed spline preserves its endpoint seam");

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
    for (real invalid : {0.0, -1.0,
                         std::numeric_limits<real>::infinity(),
                         std::numeric_limits<real>::quiet_NaN()}) {
        nurbspath::numerical_settings<real> invalid_settings;
        invalid_settings.residual_tolerance = invalid;
        bool rejected_residual = false;
        try {
            invalid_settings.validate();
        } catch (const std::invalid_argument&) {
            rejected_residual = true;
        }
        check(rejected_residual,
              "numerical settings require finite positive residual tolerance");
        invalid_settings = {};
        invalid_settings.parameter_tolerance = invalid;
        bool rejected_parameter = false;
        try {
            invalid_settings.validate();
        } catch (const std::invalid_argument&) {
            rejected_parameter = true;
        }
        check(rejected_parameter,
              "numerical settings require finite positive parameter tolerance");
    }
    return finish("09_test_float_and_detail");
}
