#include <nurbspath/config.hpp>
#include <nurbspath/nurbspath.hpp>

#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

namespace {

using real = double;

[[nodiscard]] nurbspath::nurbs_spline2<real> make_path2() {
    return nurbspath::nurbs_spline2<real>::interpolate(
        {{-1.2, -0.5}, {-0.5, 0.5}, {0.3, 0.2}, {1.1, 0.8}},
        {0.0, 1.0, 2.1, 3.4},
        3);
}

void write_scene(
    const nurbspath::svg_view3<real>& view,
    const std::string& file_name,
    const nurbspath::nurbs_spline3<real>& path,
    const nurbspath::nurbs_spline2<real>& path2) {
    nurbspath::svg_graphics_options<real> options;
    options.line_width = 1.5;
    options.spline_segment_count = 160;
    options.sphere_segment_count = 120;

    nurbspath::svg_document3<real> document(view, options);
    document.add(nurbspath::ray3<real>(
        {-2.0, -1.0, 0.0}, {1.0, 0.35, 0.2}));
    document.add(nurbspath::plane3<real>(
        {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}));
    document.add(nurbspath::sphere3<real>({1.2, 0.8, 0.7}, 0.65));
    document.add(path);
    document.add(nurbspath::point3<real>{-1.5, 1.0, 0.35});

    // A separate 2D world is embedded only for visualization through this
    // explicitly selected plane. The add(plane, entity) overloads use the same
    // pale colors as their projected 3D counterparts and patterned 2D lines.
    const nurbspath::plane3<real> drawing_plane(
        {-0.8, 0.3, 1.6}, {0.2, -0.5, 1.0}, {1.0, 0.0, 0.0});
    document.add(drawing_plane);
    document.add(drawing_plane, nurbspath::point2<real>{-1.0, 0.75});
    document.add(
        drawing_plane,
        nurbspath::ray2<real>({-1.3, -0.8}, {1.0, 0.25}));
    document.add(
        drawing_plane,
        nurbspath::circle2<real>({0.6, -0.45}, 0.45));
    document.add(drawing_plane, path2);

    std::ofstream output(file_name);
    if (!output) {
        throw std::runtime_error("could not open SVG output file: " + file_name);
    }
    document.write(output);
}

void write_flat_scene(
    const std::string& file_name,
    const nurbspath::nurbs_spline2<real>& path) {
    const nurbspath::svg_view2<real> view(
        {0.0, 0.0}, 4.5, 3.5, 960, 720);
    nurbspath::svg_graphics_options2<real> options;
    options.line_width = 1.5;
    options.spline_segment_count = 160;

    // This canvas accepts native 2D entities only and draws every line solid.
    nurbspath::svg_canvas2<real> canvas(view, options);
    canvas.add(nurbspath::ray2<real>({-1.7, -1.0}, {1.0, 0.3}));
    canvas.add(nurbspath::circle2<real>({0.8, -0.3}, 0.6));
    canvas.add(path);
    canvas.add(nurbspath::point2<real>{-1.4, 0.9});

    std::ofstream output(file_name);
    if (!output) {
        throw std::runtime_error("could not open SVG output file: " + file_name);
    }
    canvas.write(output);
}

} // namespace

int main() {
    const auto path = nurbspath::nurbs_spline3<real>::interpolate(
        {{-1.8, -0.8, 0.2},
         {-0.8, 0.6, 0.9},
         {0.3, 0.2, 1.3},
         {1.4, 1.2, 0.5},
         {2.0, 0.1, 0.2}},
        {0.0, 1.2, 2.5, 4.0, 5.4},
        3);
    const auto path2 = make_path2();

    const nurbspath::point3<real> eyepoint{5.0, -7.0, 4.5};
    const nurbspath::point3<real> lookatpoint{0.2, 0.2, 0.5};

    write_scene(
        nurbspath::svg_view3<real>::orthographic(
            eyepoint, lookatpoint, 6.0, 4.5, 960, 720),
        "nurbspath_orthographic.svg",
        path,
        path2);
    write_scene(
        nurbspath::svg_view3<real>::perspective(
            eyepoint,
            lookatpoint,
            std::numbers::pi_v<real> / 4.0,
            960,
            720,
            0.05),
        "nurbspath_perspective.svg",
        path,
        path2);
    write_flat_scene("nurbspath_2d.svg", path2);

    std::cout << "Wrote nurbspath_orthographic.svg and "
                 "nurbspath_perspective.svg and nurbspath_2d.svg\n";
}
