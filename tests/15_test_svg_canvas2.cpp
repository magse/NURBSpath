#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <limits>
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

template <typename CANVAS, typename ENTITY>
concept canvas_addable = requires(CANVAS& canvas, const ENTITY& entity) {
    canvas.add(entity);
};

template <typename VIEW, typename ENTITY>
concept flat_svg_renderable = requires(const VIEW& view, const ENTITY& entity) {
    nurbspath::to_svg(view, entity);
};

} // namespace

int main() {
    using namespace test_support;

    const nurbspath::svg_view2<real> view(
        point2<real>::origin(), 8.0, 6.0, 800, 600);
    nurbspath::svg_graphics_options2<real> options;
    options.line_width = 2.0;
    options.spline_segment_count = 10;

    const ray2<real> ray({-2.0, -1.0}, {1.0, 0.5});
    const circle2<real> circle({1.0, -1.0}, 1.0);
    const auto spline = make_line2({-2.0, 1.0}, {2.0, 2.0});

    nurbspath::svg_canvas2<real> canvas(view, options);
    canvas.add(point2<real>{1.0, 2.0});
    canvas.add(ray);
    canvas.add(circle);
    canvas.add(spline);
    const std::string svg = canvas.svg();

    check(svg.starts_with("<?xml"), "flat 2D canvas has XML declaration");
    check(count_occurrences(svg, "class=\"nurbspath_canvas_axis_x\"") == 1 &&
              count_occurrences(svg, "class=\"nurbspath_canvas_axis_y\"") == 1,
          "flat 2D canvas includes unit XY axes");
    check(svg.find("nurbspath_world_axis_z") == std::string::npos &&
              svg.find(">Z</text>") == std::string::npos,
          "flat 2D canvas has no Z axis");
    check(svg.find("class=\"nurbspath_canvas_point2\" cx=\"500\" cy=\"100\"") !=
              std::string::npos,
          "flat canvas maps native point2 directly");
    check(svg.find("class=\"nurbspath_canvas_ray2\"") != std::string::npos &&
              svg.find("#e7a0a0") != std::string::npos,
          "flat canvas renders solid pale-red ray2");
    check(svg.find("class=\"nurbspath_canvas_circle2\"") != std::string::npos &&
              svg.find("r=\"100\"") != std::string::npos &&
              svg.find("#8fc5e8") != std::string::npos,
          "flat canvas renders native pale-blue circle2");
    check(count_occurrences(svg, "class=\"nurbspath_canvas_spline2\"") == 10 &&
              count_occurrences(
                  svg, "class=\"nurbspath_canvas_spline2_control_point\"") == 2 &&
              count_occurrences(
                  svg, "class=\"nurbspath_canvas_spline2_endpoint\"") == 2,
          "flat canvas renders solid spline2 geometry");
    check(svg.find("stroke-dasharray") == std::string::npos,
          "every flat 2D canvas line is solid");
    check(!contains_nonfinite_svg_number(svg),
          "flat canvas coordinates are finite");

    const std::string one_circle = nurbspath::to_svg(view, circle, options);
    check(one_circle.find("nurbspath_canvas_circle2") != std::string::npos &&
              one_circle.find("stroke-dasharray") == std::string::npos,
          "flat to_svg overload renders native 2D entity with solid line");

    const nurbspath::svg_view2<float> float_view(
        {0.0F, 0.0F}, 4.0F, 4.0F);
    const std::string unit_ray_svg = nurbspath::to_svg(
        float_view, nurbspath::ray2<float>({0.0F, 0.0F}, {1.0F, 0.0F}));
    for (const float magnitude : {1e-5F, 1e20F}) {
        const nurbspath::ray2<float> scaled_ray(
            {0.0F, 0.0F}, {magnitude, 0.0F});
        const std::string scaled_ray_svg = nurbspath::to_svg(
            float_view, scaled_ray);
        check(scaled_ray_svg == unit_ray_svg &&
                  count_occurrences(
                      scaled_ray_svg, "class=\"nurbspath_canvas_ray2\"") == 1,
              "flat ray glyph is independent of direction magnitude");
        check(scaled_ray.direction().x == magnitude,
              "flat ray rendering preserves the native parameter direction");
    }

    const std::string overflowing_circle = nurbspath::to_svg(
        view, circle2<real>({0.0, 0.0}, std::numeric_limits<real>::max()));
    check(!contains_nonfinite_svg_number(overflowing_circle) &&
              count_occurrences(
                  overflowing_circle, "class=\"nurbspath_canvas_circle2\"") == 0 &&
              count_occurrences(
                  overflowing_circle,
                  "class=\"nurbspath_canvas_circle2_center\"") == 1,
          "unrepresentable circle outlines are omitted while finite center dots remain");

    using canvas_type = nurbspath::svg_canvas2<real>;
    static_assert(canvas_addable<canvas_type, point2<real>>);
    static_assert(canvas_addable<canvas_type, ray2<real>>);
    static_assert(canvas_addable<canvas_type, circle2<real>>);
    static_assert(canvas_addable<canvas_type, nurbs_spline2<real>>);
    static_assert(!canvas_addable<canvas_type, vector2<real>>);
    static_assert(!canvas_addable<canvas_type, point3<real>>);
    static_assert(!canvas_addable<canvas_type, ray3<real>>);
    static_assert(!canvas_addable<canvas_type, sphere3<real>>);
    static_assert(!canvas_addable<canvas_type, plane3<real>>);
    static_assert(!canvas_addable<canvas_type, nurbs_spline3<real>>);
    static_assert(flat_svg_renderable<
        nurbspath::svg_view2<real>, circle2<real>>);
    static_assert(!flat_svg_renderable<
        nurbspath::svg_view2<real>, point3<real>>);

    bool rejected_view = false;
    try {
        static_cast<void>(nurbspath::svg_view2<real>(
            point2<real>::origin(), 0.0, 4.0));
    } catch (const std::invalid_argument&) {
        rejected_view = true;
    }
    check(rejected_view, "flat canvas rejects invalid viewport");

    bool rejected_options = false;
    try {
        auto invalid_options = options;
        invalid_options.spline_segment_count = 0;
        static_cast<void>(nurbspath::svg_canvas2<real>(view, invalid_options));
    } catch (const std::invalid_argument&) {
        rejected_options = true;
    }
    check(rejected_options, "flat canvas rejects invalid graphics options");

    bool rejected_overflowing_width = false;
    try {
        auto invalid_options = options;
        invalid_options.line_width = std::numeric_limits<real>::max();
        invalid_options.validate();
    } catch (const std::invalid_argument&) {
        rejected_overflowing_width = true;
    }
    check(rejected_overflowing_width,
          "flat graphics reject line widths whose derived sizes overflow");

    auto huge_options = options;
    huge_options.line_width = std::numeric_limits<real>::max() / 8.0;
    const nurbspath::svg_view2<real> far_view(
        {-std::numeric_limits<real>::max() * 0.009, 0.0}, 8.0, 6.0);
    const nurbspath::svg_canvas2<real> far_canvas(far_view, huge_options);
    const std::string far_svg = far_canvas.svg();
    check(!contains_nonfinite_svg_number(far_svg) &&
              far_svg.find("_label\"") == std::string::npos,
          "flat labels with overflowing screen offsets are omitted");

    return finish("15_test_svg_canvas2");
}
