#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <numbers>
#include <string>
#include <type_traits>

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

} // namespace

int main() {
    using namespace test_support;

    const plane3<real> plane(
        {3.0, -2.0, 5.0}, {0.0, 1.0, 1.0}, {1.0, 0.0, 0.0});
    const point2<real> point{2.0, -1.5};
    check_point(nurbspath::project(plane, point), plane.point_at(2.0, -1.5),
                1e-12, "point2 projects through plane frame");

    const vector2<real> vector{-2.0, 0.5};
    const vector3<real> projected_vector = nurbspath::project(plane, vector);
    check(projected_vector.approximately_equal(
              -2.0 * plane.u_direction() + 0.5 * plane.v_direction(), 1e-12),
          "vector2 projects through plane basis");
    check_near(projected_vector.length(), vector.length(), 1e-12,
               "orthonormal projection preserves vector length");

    const ray2<real> ray({-1.0, 0.25}, {2.0, -0.5});
    const ray3<real> projected_ray = nurbspath::project(plane, ray);
    check_point(projected_ray.evaluate(2.25),
                nurbspath::project(plane, ray.evaluate(2.25)), 1e-12,
                "ray2 projection preserves native s");

    const circle2<real> circle({0.5, -0.75}, 1.25);
    const sphere3<real> sphere = nurbspath::project(plane, circle);
    check_point(sphere.center(), nurbspath::project(plane, circle.center()), 1e-12,
                "circle2 projects center to sphere3");
    check_near(sphere.radius(), circle.radius(), 0.0,
               "circle2 projection preserves radius only");

    const nurbs_spline2<real> spline(
        {{-1.0, 0.0}, {0.0, 1.5}, {2.0, 0.25}},
        {1.0, 0.7, 1.0},
        {3.0, 3.0, 3.0, 7.0, 7.0, 7.0},
        2);
    const nurbs_spline3<real> projected_spline = nurbspath::project(plane, spline);
    check(projected_spline.weights() == spline.weights(),
          "spline projection preserves weights");
    check(projected_spline.knots() == spline.knots(),
          "spline projection preserves knots");
    check(projected_spline.degree() == spline.degree(),
          "spline projection preserves degree");
    check_near(projected_spline.tolerance(), spline.tolerance(), 0.0,
               "spline projection preserves tolerance");
    check(projected_spline.is_closed() == spline.is_closed(),
          "spline projection preserves closure");
    for (std::size_t index = 0; index < spline.control_points().size(); ++index) {
        check_point(projected_spline.control_points()[index],
                    nurbspath::project(plane, spline.control_points()[index]),
                    1e-12, "spline projection maps control points only");
    }
    check_point(projected_spline.evaluate(5.0),
                nurbspath::project(plane, spline.evaluate(5.0)), 1e-11,
                "affine spline evaluation commutes with projection");

    const nurbs_spline2<real> closed_spline(
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
        1,
        true);
    const auto projected_closed = nurbspath::project(plane, closed_spline);
    check(projected_closed.is_closed(),
          "closed spline projection retains its seam state");
    check_point(projected_closed.get_start(), projected_closed.get_end(), 1e-12,
                "projected closed spline retains coincident endpoints");

    const auto view = nurbspath::svg_view3<real>::orthographic(
        {8.0, -10.0, 9.0}, plane.origin(), 10.0, 8.0, 800, 640);
    nurbspath::svg_graphics_options<real> options;
    options.spline_segment_count = 9;
    options.sphere_segment_count = 24;
    nurbspath::svg_document3<real> document(view, options);
    document.add(plane, point);
    document.add(plane, ray);
    document.add(plane, circle);
    document.add(plane, spline);
    const std::string svg = document.svg();
    check(svg.find("class=\"nurbspath_point2\"") != std::string::npos,
          "SVG add plane-point2 uses point style");
    check(svg.find("class=\"nurbspath_ray2\"") != std::string::npos,
          "SVG add plane-ray2 uses pale-red patterned ray style");
    check(svg.find("nurbspath_circle2_outline") != std::string::npos,
          "SVG add plane-circle2 uses pale-blue patterned sphere style");
    check(count_occurrences(svg, "class=\"nurbspath_spline2\"") == 9,
          "SVG add plane-spline2 uses requested tessellation");
    check(svg.find("stroke-dasharray=\"7.5 3.75 0.75 3.75\"") !=
              std::string::npos,
          "projected 2D primary strokes are dash-dotted");
    check(svg.find("stroke-dasharray=\"0.75 3.75\"") != std::string::npos,
          "projected 2D auxiliary strokes are dotted");

    const std::string single = nurbspath::to_svg(view, plane, point, options);
    check(single.find("class=\"nurbspath_point2\"") != std::string::npos,
          "plane-aware to_svg renders 2D entity");

    static_assert(std::is_same_v<
        decltype(nurbspath::project(plane, point)), point3<real>>);
    static_assert(std::is_same_v<
        decltype(nurbspath::project(plane, ray)), ray3<real>>);
    static_assert(std::is_same_v<
        decltype(nurbspath::project(plane, circle)), sphere3<real>>);
    static_assert(std::is_same_v<
        decltype(nurbspath::project(plane, spline)), nurbs_spline3<real>>);

    const nurbspath::plane3<float> float_plane(
        nurbspath::point3<float>{0.0F, 0.0F, 0.0F},
        nurbspath::vector3<float>{0.0F, 0.0F, 1.0F},
        nurbspath::vector3<float>{1.0F, 0.0F, 0.0F});
    const auto float_sphere = nurbspath::project(
        float_plane,
        nurbspath::circle2<float>({1.0F, 2.0F}, 0.5F));
    check_near(static_cast<real>(float_sphere.radius()), 0.5, 0.0,
               "2D projection API supports float");
    return finish("13_test_2d_projection_graphics");
}
