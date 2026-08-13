#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/circle2.hpp"
#include "nurbspath/nurbs_spline2.hpp"
#include "nurbspath/nurbs_spline3.hpp"
#include "nurbspath/plane3.hpp"
#include "nurbspath/sphere3.hpp"
#include "nurbspath/utility.hpp"

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>

namespace nurbspath {

/**
 * @brief Detailed 2D point-to-circle distance result.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct point_circle_distance2 {
    REAL distance = REAL(0); ///< Nonnegative tolerance-adjusted 2D distance.
    point2<REAL> closest_point; ///< Closest point on the 2D circle.
    REAL u = REAL(0); ///< Circle angle at closest_point in `[0,2*pi)`.
};

/**
 * @brief Detailed 2D point-to-spline distance result.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct point_spline_distance2 {
    REAL distance = REAL(0); ///< Nonnegative tolerance-adjusted 2D distance.
    point2<REAL> closest_point; ///< Closest sampled-and-refined spline point.
    REAL s = REAL(0); ///< Spline parameter at closest_point.
};

template <std::floating_point REAL>
/**
 * @brief Detailed point-to-surface distance result.
 * @tparam REAL Floating-point scalar type.
 */
struct point_surface_distance3 {
    REAL distance = REAL(0); ///< Nonnegative tolerance-adjusted distance.
    point3<REAL> closest_point; ///< Closest point on the queried surface.
    REAL u = REAL(0); ///< Surface u parameter at closest_point.
    REAL v = REAL(0); ///< Surface v parameter at closest_point.
};

template <std::floating_point REAL>
/**
 * @brief Detailed point-to-spline distance result.
 * @tparam REAL Floating-point scalar type.
 */
struct point_spline_distance3 {
    REAL distance = REAL(0); ///< Nonnegative tolerance-adjusted distance.
    point3<REAL> closest_point; ///< Closest point found on the spline.
    REAL s = REAL(0); ///< Spline parameter at closest_point.
};

/**
 * @brief Compute radial point-to-circle distance information in 2D.
 * @tparam REAL Floating-point scalar type.
 * @param point Query point in the independent 2D world.
 * @param circle Target circle in the same 2D world.
 * @param tolerance Positive distance snapped to zero on contact.
 * @return 2D distance, closest circle point, and circle angle.
 * @throws std::invalid_argument When tolerance is not finite and positive.
 */
template <std::floating_point REAL>
[[nodiscard]] point_circle_distance2<REAL> distance_to_circle(
    const point2<REAL>& point,
    const circle2<REAL>& circle,
    REAL tolerance = REAL(1e-8)) {
    if (!(tolerance > REAL(0)) || !std::isfinite(tolerance)) {
        throw std::invalid_argument("distance tolerance must be finite and positive");
    }

    const vector2<REAL> radial = point - circle.center();
    const REAL center_distance = radial.length();
    const vector2<REAL> direction = center_distance == REAL(0)
        ? vector2<REAL>::unit_x()
        : radial / center_distance;
    const point2<REAL> closest = circle.center() + circle.radius() * direction;
    const REAL measured = std::abs(center_distance - circle.radius());
    return {
        measured <= tolerance ? REAL(0) : measured,
        closest,
        circle.parameter_of(closest)};
}

/**
 * @brief Numerically find the sampled global 2D point-to-spline minimum.
 *
 * Every sampled local basin is refined by golden-section minimization.
 * Coverage therefore depends on `settings.sample_count`. No plane or 3D
 * projection participates in this calculation.
 *
 * @tparam REAL Floating-point scalar type.
 * @param point Query point in the independent 2D world.
 * @param spline Target NURBS curve in the same 2D world.
 * @param settings Numerical tolerances, iterations, and sampling density.
 * @return 2D distance, closest spline point found, and spline parameter.
 * @throws std::invalid_argument When numerical settings are invalid.
 */
template <std::floating_point REAL>
[[nodiscard]] point_spline_distance2<REAL> distance_to_spline(
    const point2<REAL>& point,
    const nurbs_spline2<REAL>& spline,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    const auto squared_distance_at = [&](REAL s) {
        return distance_squared(point, spline.evaluate(s));
    };

    const std::size_t count = settings.sample_count;
    std::vector<REAL> parameters(count + 1);
    std::vector<REAL> values(count + 1);
    REAL best_s = spline.s_min();
    REAL best_value = squared_distance_at(best_s);

    for (std::size_t index = 0; index <= count; ++index) {
        const REAL fraction = static_cast<REAL>(index) / static_cast<REAL>(count);
        parameters[index] = spline.s_min() +
                            fraction * (spline.s_max() - spline.s_min());
        values[index] = squared_distance_at(parameters[index]);
        if (values[index] < best_value) {
            best_value = values[index];
            best_s = parameters[index];
        }
    }

    for (std::size_t index = 1; index < count; ++index) {
        if (values[index] <= values[index - 1] &&
            values[index] <= values[index + 1]) {
            const REAL candidate = golden_section_minimize<REAL>(
                squared_distance_at,
                parameters[index - 1],
                parameters[index + 1],
                settings);
            const REAL candidate_value = squared_distance_at(candidate);
            if (candidate_value < best_value) {
                best_value = candidate_value;
                best_s = candidate;
            }
        }
    }

    const REAL measured = std::sqrt(std::max(best_value, REAL(0)));
    return {
        measured <= settings.residual_tolerance ? REAL(0) : measured,
        spline.evaluate(best_s),
        best_s};
}

/**
 * @brief Compute orthogonal point-to-plane distance information.
 * @tparam REAL Floating-point scalar type.
 * @param point World-space query point.
 * @param plane Infinite target plane.
 * @param tolerance Positive distance snapped to zero on contact.
 * @return Distance, closest point, and plane parameters.
 * @throws std::invalid_argument When tolerance is not positive.
 */
template <std::floating_point REAL>
[[nodiscard]] point_surface_distance3<REAL> distance_to_plane(
    const point3<REAL>& point,
    const plane3<REAL>& plane,
    REAL tolerance = REAL(1e-8)) {
    if (!(tolerance > REAL(0))) {
        throw std::invalid_argument("distance tolerance must be positive");
    }
    const point3<REAL> closest = plane.project(point);
    const auto [u, v] = plane.parameters_of(closest);
    const REAL measured = std::abs(plane.signed_distance_to(point));
    return {measured <= tolerance ? REAL(0) : measured, closest, u, v};
}

/**
 * @brief Compute radial point-to-sphere distance information.
 * @tparam REAL Floating-point scalar type.
 * @param point World-space query point.
 * @param sphere Target sphere.
 * @param tolerance Positive distance snapped to zero on contact.
 * @return Distance, closest point, and spherical parameters.
 * @throws std::invalid_argument When tolerance is not positive.
 */
template <std::floating_point REAL>
[[nodiscard]] point_surface_distance3<REAL> distance_to_sphere(
    const point3<REAL>& point,
    const sphere3<REAL>& sphere,
    REAL tolerance = REAL(1e-8)) {
    if (!(tolerance > REAL(0))) {
        throw std::invalid_argument("distance tolerance must be positive");
    }

    const vector3<REAL> radial = point - sphere.center();
    const REAL center_distance = radial.length();
    // Every sphere point is equally close to the center; choose (u,v)=(0,0)
    // deterministically instead of attempting to normalize a zero vector.
    const vector3<REAL> direction = center_distance == REAL(0)
        ? vector3<REAL>::unit_x()
        : radial / center_distance;
    const point3<REAL> closest = sphere.center() + sphere.radius() * direction;
    const REAL angular_tolerance = std::min(tolerance, sphere.radius() * REAL(0.5));
    const auto [u, v] = sphere.parameters_of(closest, angular_tolerance);
    const REAL measured = std::abs(center_distance - sphere.radius());
    return {measured <= tolerance ? REAL(0) : measured, closest, u, v};
}

/**
 * @brief Numerically find the sampled global point-to-spline minimum.
 *
 * Every sampled local basin is refined by golden-section minimization. Coverage
 * therefore depends on `settings.sample_count`.
 *
 * @tparam REAL Floating-point scalar type.
 * @param point World-space query point.
 * @param spline Target NURBS curve.
 * @param settings Numerical tolerances, iterations, and sampling density.
 * @return Distance, closest point found, and spline parameter.
 * @throws std::invalid_argument When numerical settings are invalid.
 */
template <std::floating_point REAL>
[[nodiscard]] point_spline_distance3<REAL> distance_to_spline(
    const point3<REAL>& point,
    const nurbs_spline3<REAL>& spline,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    const auto squared_distance_at = [&](REAL s) {
        return distance_squared(point, spline.evaluate(s));
    };

    const std::size_t count = settings.sample_count;
    std::vector<REAL> parameters(count + 1);
    std::vector<REAL> values(count + 1);
    REAL best_s = spline.s_min();
    REAL best_value = squared_distance_at(best_s);

    for (std::size_t index = 0; index <= count; ++index) {
        const REAL fraction = static_cast<REAL>(index) / static_cast<REAL>(count);
        parameters[index] = spline.s_min() +
                            fraction * (spline.s_max() - spline.s_min());
        values[index] = squared_distance_at(parameters[index]);
        if (values[index] < best_value) {
            best_value = values[index];
            best_s = parameters[index];
        }
    }

    for (std::size_t index = 1; index < count; ++index) {
        if (values[index] <= values[index - 1] &&
            values[index] <= values[index + 1]) {
            const REAL candidate = golden_section_minimize<REAL>(
                squared_distance_at,
                parameters[index - 1],
                parameters[index + 1],
                settings);
            const REAL candidate_value = squared_distance_at(candidate);
            if (candidate_value < best_value) {
                best_value = candidate_value;
                best_s = candidate;
            }
        }
    }

    const REAL measured = std::sqrt(std::max(best_value, REAL(0)));
    return {
        measured <= settings.residual_tolerance ? REAL(0) : measured,
        spline.evaluate(best_s),
        best_s
    };
}

/**
 * @brief Compute scalar point-to-plane distance.
 * @tparam REAL Floating-point scalar type.
 * @param point World-space query point.
 * @param plane Infinite target plane.
 * @param tolerance Positive contact tolerance.
 * @return Tolerance-adjusted nonnegative distance.
 */
template <std::floating_point REAL>
[[nodiscard]] REAL distance(
    const point3<REAL>& point,
    const plane3<REAL>& plane,
    REAL tolerance = REAL(1e-8)) {
    return distance_to_plane(point, plane, tolerance).distance;
}

/**
 * @brief Compute scalar point-to-sphere distance.
 * @tparam REAL Floating-point scalar type.
 * @param point World-space query point.
 * @param sphere Target sphere.
 * @param tolerance Positive contact tolerance.
 * @return Tolerance-adjusted nonnegative distance.
 */
template <std::floating_point REAL>
[[nodiscard]] REAL distance(
    const point3<REAL>& point,
    const sphere3<REAL>& sphere,
    REAL tolerance = REAL(1e-8)) {
    return distance_to_sphere(point, sphere, tolerance).distance;
}

/**
 * @brief Compute scalar numerical point-to-spline distance.
 * @tparam REAL Floating-point scalar type.
 * @param point World-space query point.
 * @param spline Target NURBS curve.
 * @param settings Numerical controls for sampled minimization.
 * @return Tolerance-adjusted nonnegative distance.
 */
template <std::floating_point REAL>
[[nodiscard]] REAL distance(
    const point3<REAL>& point,
    const nurbs_spline3<REAL>& spline,
    const numerical_settings<REAL>& settings = {}) {
    return distance_to_spline(point, spline, settings).distance;
}

/**
 * @brief Compute scalar point-to-circle distance entirely in 2D.
 * @tparam REAL Floating-point scalar type.
 * @param point Query point in the independent 2D world.
 * @param circle Target circle in the same 2D world.
 * @param tolerance Positive contact tolerance.
 * @return Tolerance-adjusted nonnegative 2D distance.
 */
template <std::floating_point REAL>
[[nodiscard]] REAL distance(
    const point2<REAL>& point,
    const circle2<REAL>& circle,
    REAL tolerance = REAL(1e-8)) {
    return distance_to_circle(point, circle, tolerance).distance;
}

/**
 * @brief Compute scalar numerical point-to-spline distance entirely in 2D.
 * @tparam REAL Floating-point scalar type.
 * @param point Query point in the independent 2D world.
 * @param spline Target spline in the same 2D world.
 * @param settings Numerical controls for sampled minimization.
 * @return Tolerance-adjusted nonnegative 2D distance.
 */
template <std::floating_point REAL>
[[nodiscard]] REAL distance(
    const point2<REAL>& point,
    const nurbs_spline2<REAL>& spline,
    const numerical_settings<REAL>& settings = {}) {
    return distance_to_spline(point, spline, settings).distance;
}

} // namespace nurbspath
