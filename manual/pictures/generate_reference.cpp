#include <nurbspath/config.hpp>
#include <nurbspath/nurbspath.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using real = double;
namespace np = nurbspath;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::ofstream output_file(const std::filesystem::path& directory,
                          const std::string& name) {
    std::ofstream output(directory / name);
    output.exceptions(std::ios::badbit | std::ios::failbit);
    output << std::scientific << std::setprecision(4);
    return output;
}

template <typename CURVE>
void write_curve(const std::filesystem::path& directory,
                 const std::string& name, const CURVE& curve) {
    auto output = output_file(directory, name);
    output << "s x y\n";
    constexpr std::size_t count = 240;
    for (std::size_t index = 0; index <= count; ++index) {
        const real s = curve.s_min() + (curve.s_max() - curve.s_min()) *
            static_cast<real>(index) / static_cast<real>(count);
        const auto point = curve.evaluate(s);
        require(std::isfinite(point.x) && std::isfinite(point.y),
                "nonfinite reference curve");
        output << s << ' ' << point.x << ' ' << point.y << '\n';
    }
}

void write_references(const std::filesystem::path& directory) {
    const np::nurbs_spline2<real> spline(
        {{0.0, 0.0}, {1.0, 2.0}, {2.0, -1.0}, {3.0, 2.0}, {4.0, 0.0}},
        {1.0, 1.0, 0.7, 1.5, 1.0},
        {0.0, 0.0, 0.0, 0.0, 2.0, 5.0, 5.0, 5.0, 5.0}, 3);
    auto curve_output = output_file(directory, "spline_curve.dat");
    curve_output << "s x y dx dy ddx ddy dddx dddy curvature\n";
    constexpr std::size_t count = 240;
    for (std::size_t index = 0; index <= count; ++index) {
        const real s = 5.0 * static_cast<real>(index) / static_cast<real>(count);
        const auto derivatives = spline.derivatives_at(s);
        const auto third = spline.third_derivative(s);
        const real curvature = spline.curvature(s);
        require(std::isfinite(curvature), "nonfinite reference curvature");
        curve_output << s << ' ' << derivatives.point.x << ' '
            << derivatives.point.y << ' ' << derivatives.first.x << ' '
            << derivatives.first.y << ' ' << derivatives.second.x << ' '
            << derivatives.second.y << ' ' << third.x << ' ' << third.y
            << ' ' << curvature << '\n';
    }
    auto controls = output_file(directory, "spline_controls.dat");
    controls << "x y weight\n";
    for (std::size_t index = 0; index < spline.get_control_points().size(); ++index) {
        const auto point = spline.get_control_point(index);
        controls << point.x << ' ' << point.y << ' '
                 << spline.get_weight(index) << '\n';
    }

    const std::vector<np::point2<real>> samples{
        {0.0, 0.0}, {1.0, 1.2}, {2.0, 0.2}, {3.0, 1.5}, {4.0, 1.0}};
    const std::vector<real> stations{2.0, 3.2, 4.7, 6.5, 8.0};
    const auto interpolation = np::nurbs_spline2<real>::interpolate(
        samples, stations, 3);
    write_curve(directory, "interpolation_curve.dat", interpolation);
    auto measured = output_file(directory, "interpolation_samples.dat");
    measured << "s x y\n";
    for (std::size_t index = 0; index < samples.size(); ++index) {
        require(np::distance(interpolation.evaluate(stations[index]),
                             samples[index]) < 1e-10,
                "global interpolation missed a reference sample");
        measured << stations[index] << ' ' << samples[index].x << ' '
                 << samples[index].y << '\n';
    }

    const np::nurbs_spline2<real> quarter_circle(
        {{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}},
        {1.0, std::sqrt(0.5), 1.0},
        {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}, 2);
    write_curve(directory, "rational_circle.dat", quarter_circle);
    for (std::size_t index = 0; index <= 100; ++index) {
        const auto point = quarter_circle.evaluate(static_cast<real>(index) / 100.0);
        require(std::abs(point.x * point.x + point.y * point.y - 1.0) < 1e-12,
                "rational circular arc failed its radius check");
    }

    np::numerical_settings<real> settings;
    settings.residual_tolerance = 1e-10;
    settings.parameter_tolerance = 1e-12;
    settings.sample_count = 512;
    const np::circle2<real> circle({0.0, 0.0}, 1.0);
    const auto crossing = np::intersect_ray_circle(
        np::ray2<real>({-3.0, 0.0}, {2.0, 0.0}), circle, settings);
    const auto tangent = np::intersect_ray_circle(
        np::ray2<real>({-2.0, 1.0}, {1.0, 0.0}), circle, settings);
    require(crossing.points.size() == 2, "crossing reference lost a contact");
    require(tangent.points.size() == 1, "tangent reference is not isolated once");
    auto crossing_output = output_file(directory, "intersection_crossing.dat");
    crossing_output << "s x y\n";
    for (const auto& hit : crossing.points) {
        crossing_output << hit.s << ' ' << hit.point.x << ' ' << hit.point.y << '\n';
    }
    auto tangent_output = output_file(directory, "intersection_tangent.dat");
    tangent_output << "s x y\n";
    for (const auto& hit : tangent.points) {
        tangent_output << hit.s << ' ' << hit.point.x << ' ' << hit.point.y << '\n';
    }

    const np::point2<real> query{2.0, 2.5};
    const auto closest = np::distance_to_spline(query, spline, settings);
    auto distance_output = output_file(directory, "distance_reference.dat");
    distance_output << "x y\n" << query.x << ' ' << query.y << '\n'
                    << closest.closest_point.x << ' ' << closest.closest_point.y << '\n';
    require(std::abs(np::distance(query, closest.closest_point) - closest.distance) < 1e-10,
            "closest-point reference disagrees with its reported distance");

    const auto root_function = [](real x) {
        return (x + 0.8) * np::square(x - 0.3) * (x - 1.2);
    };
    const auto roots = np::find_roots<real>(root_function, -1.5, 1.6, settings);
    require(roots.size() == 3, "root reference missed an ordinary or tangent root");
    auto root_output = output_file(directory, "algorithmroots.dat");
    root_output << "x y\n";
    for (std::size_t index = 0; index <= 300; ++index) {
        const real x = -1.5 + 3.1 * static_cast<real>(index) / 300.0;
        root_output << x << ' ' << root_function(x) << '\n';
    }
    auto root_points = output_file(directory, "root_points.dat");
    root_points << "x y\n";
    for (const real root : roots) {
        root_points << root << ' ' << 0.0 << '\n';
    }

    const auto solution = np::solve_linear_system<real>(
        {{2.0, 1.0}, {1.0, 3.0}}, {5.0, 7.0});
    require(std::abs(solution[0] - 1.6) < 1e-12 &&
            std::abs(solution[1] - 1.8) < 1e-12,
            "linear system reference is incorrect");
    const auto tangent_at_two = spline.tangent(2.0);
    const auto point_at_two = spline.evaluate(2.0);
    const auto circle_distance = np::distance_to_circle(np::point2<real>{2.0, 1.5}, circle);
    const np::plane3<real> plane({0.0, 0.0, 1.0}, {0.0, 0.0, 1.0});
    const auto plane_distance = np::distance_to_plane(np::point3<real>{1.0, 1.0, 3.0}, plane);
    require(std::abs(plane_distance.distance - 2.0) < 1e-12,
            "plane distance reference is incorrect");

    auto values = output_file(directory, "reference_values.tex");
    values << "% Generated by generate_reference.cpp; display values only.\n"
           << "\\def\\ManualVersion{" << NURBSPATH_VERSION_STRING << "}\n";
    const auto macro = [&](const char* name, real value) {
        values << "\\def\\" << name << '{' << value << "}\n";
    };
    macro("RefClosestS", closest.s);
    macro("RefClosestDistance", closest.distance);
    macro("RefArcLength", spline.approximate_arc_length(1024));
    macro("RefPolygonLength", spline.get_polygon_length());
    macro("RefCurvature", spline.curvature(2.0));
    macro("RefCurveX", point_at_two.x);
    macro("RefCurveY", point_at_two.y);
    macro("RefTangentX", tangent_at_two.x);
    macro("RefTangentY", tangent_at_two.y);
    macro("RefCircleDistance", circle_distance.distance);
    macro("RefFirstRoot", roots[0]);
    macro("RefSecondRoot", roots[1]);
    macro("RefThirdRoot", roots[2]);
    macro("RefLinearX", solution[0]);
    macro("RefLinearY", solution[1]);

    // Same-world array conversion and explicit embedding are checked here too.
    const np::nurbs_arr_spline2<real> array(spline);
    const auto restored = array.to_nurbs_spline();
    require(np::distance(restored.evaluate(2.0), point_at_two) < 1e-12,
            "array-backed conversion changed reference geometry");
    const auto embedded = np::project(plane, spline);
    const auto expected = np::project(plane, spline.evaluate(2.0));
    require(np::distance(embedded.evaluate(2.0), expected) < 1e-12,
            "plane embedding changed reference parameters");
    std::cout << "Reference geometry generated and checked using nurbspath "
              << NURBSPATH_VERSION_STRING << '\n';
}

} // namespace

int main(int argc, char** argv) {
    try {
        const std::filesystem::path directory = argc > 1 ? argv[1] : ".";
        std::filesystem::create_directories(directory);
        write_references(directory);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
