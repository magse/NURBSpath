#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <numbers>
#include <stdexcept>
#include <string>

namespace {

std::size_t count_occurrences(const std::string& text, std::string_view pattern) {
    std::size_t count = 0;
    std::size_t position = 0;
    while ((position = text.find(pattern, position)) != std::string::npos) {
        ++count;
        position += pattern.size();
    }
    return count;
}

bool contains_nonfinite_svg_number(const std::string& text) {
    return text.find("=\"nan") != std::string::npos ||
           text.find("=\"-nan") != std::string::npos ||
           text.find("=\"inf") != std::string::npos ||
           text.find("=\"-inf") != std::string::npos;
}

} // namespace

int main() {
    using namespace test_support;

    const point3<real> eye{5.0, -7.0, 4.0};
    const point3<real> look_at{0.0, 0.0, 0.5};
    const auto orthographic_view = nurbspath::svg_view3<real>::orthographic(
        eye, look_at, 8.0, 6.0, 800, 600);
    const auto perspective_view = nurbspath::svg_view3<real>::perspective(
        eye, look_at, std::numbers::pi_v<real> / 3.0, 800, 600, 0.01);

    nurbspath::svg_graphics_options<real> options;
    options.line_width = 2.0;
    options.spline_segment_count = 12;
    options.sphere_segment_count = 24;
    check_near(options.plane_square_side_length, 0.25, 1e-12,
               "plane square defaults to quarter-unit sides");
    check_near(options.plane_parameter_axis_length, 0.5, 1e-12,
               "plane axes default to half-unit lines");

    const auto path = make_line({-1.5, -0.4, 0.2}, {1.5, 0.8, 1.1});
    const ray3<real> ray({-1.0, -1.0, 0.0}, {1.0, 0.5, 0.25});
    const sphere3<real> sphere({0.5, 0.3, 0.8}, 0.7);
    const plane3<real> plane(
        {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    const point3<real> point{0.0, 0.0, 0.0};

    nurbspath::svg_document3<real> orthographic(orthographic_view, options);
    orthographic.add(ray);
    orthographic.add(sphere);
    orthographic.add(plane);
    orthographic.add(path);
    orthographic.add(point);
    const std::string svg = orthographic.svg();

    check(svg.starts_with("<?xml"), "SVG has XML declaration");
    check(count_occurrences(svg, "class=\"nurbspath_world_axis_x\"") == 1 &&
              count_occurrences(svg, "class=\"nurbspath_world_axis_y\"") == 1 &&
              count_occurrences(svg, "class=\"nurbspath_world_axis_z\"") == 1,
          "SVG includes XYZ system");
    check(svg.find(">X</text>") != std::string::npos &&
              svg.find(">Y</text>") != std::string::npos &&
              svg.find(">Z</text>") != std::string::npos,
          "SVG axes have labels");
    check(svg.find("class=\"nurbspath_point\"") != std::string::npos,
          "SVG renders point");
    check(svg.find("class=\"nurbspath_ray\"") != std::string::npos &&
              svg.find("nurbspath_ray_positive_direction") != std::string::npos &&
              svg.find("nurbspath_ray_origin") != std::string::npos,
          "SVG renders complete ray glyph");
    check(svg.find("nurbspath_sphere_outline") != std::string::npos &&
              svg.find("nurbspath_sphere_equator_hidden") != std::string::npos &&
              svg.find("nurbspath_sphere_center") != std::string::npos,
          "SVG renders complete sphere glyph");
    check(count_occurrences(svg, "nurbspath_plane_square") == 4 &&
              svg.find("nurbspath_plane_u_direction") != std::string::npos &&
              svg.find("nurbspath_plane_v_direction") != std::string::npos,
          "SVG renders complete plane glyph");
    check(count_occurrences(svg, "class=\"nurbspath_spline\"") == 12 &&
              count_occurrences(
                  svg, "class=\"nurbspath_spline_control_point\"") == 2 &&
              count_occurrences(
                  svg, "class=\"nurbspath_spline_endpoint\"") == 2,
          "SVG renders complete spline glyph");
    check(!contains_nonfinite_svg_number(svg), "orthographic coordinates are finite");

    nurbspath::svg_document3<real> perspective(perspective_view, options);
    perspective.add(ray);
    perspective.add(sphere);
    perspective.add(plane);
    perspective.add(path);
    const std::string perspective_svg = perspective.svg();
    check(!contains_nonfinite_svg_number(perspective_svg),
          "perspective coordinates are finite");
    check(perspective_svg != svg, "projections differ");

    const std::string one_point = nurbspath::to_svg(orthographic_view, point, options);
    check(count_occurrences(one_point, "nurbspath_point") == 1,
          "single-entity convenience output");

    bool rejected_equal_points = false;
    try {
        static_cast<void>(nurbspath::svg_view3<real>::orthographic(
            point3<real>::origin(), point3<real>::origin(), 4.0, 3.0));
    } catch (const std::invalid_argument&) {
        rejected_equal_points = true;
    }
    check(rejected_equal_points, "camera rejects equal points");

    return finish("08_test_svg_graphics");
}
