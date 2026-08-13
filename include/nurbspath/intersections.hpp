#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/circle2.hpp"
#include "nurbspath/nurbs_spline2.hpp"
#include "nurbspath/nurbs_spline3.hpp"
#include "nurbspath/plane3.hpp"
#include "nurbspath/ray2.hpp"
#include "nurbspath/ray3.hpp"
#include "nurbspath/sphere3.hpp"
#include "nurbspath/utility.hpp"

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <vector>

namespace nurbspath {

/** @brief Classification of a geometric intersection set. */
enum class intersection_kind {
    none, ///< No contact within the queried parameter domain.
    discrete, ///< One or more isolated contacts are listed.
    coincident ///< A continuous overlap cannot be represented by finite points.
};

/**
 * @brief 2D contact with parameters of both participating entities.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct intersection2 {
    point2<REAL> point; ///< Contact point in the independent 2D world.
    REAL s = REAL(0); ///< Ray or spline parameter.
    REAL u = REAL(0); ///< Circle angle in `[0,2*pi)`.
    REAL residual = REAL(0); ///< Absolute 2D circle residual after refinement.
};

/**
 * @brief Complete result of one two-dimensional intersection query.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct intersection_result2 {
    intersection_kind kind = intersection_kind::none; ///< Set classification.
    std::vector<intersection2<REAL>> points; ///< Isolated contacts sorted by s.

    /** @brief Test for isolated or coincident contact. @return True unless kind is none. */
    [[nodiscard]] bool has_intersection() const noexcept {
        return kind != intersection_kind::none;
    }

    /** @brief Test for continuous overlap. @return True when kind is coincident. */
    [[nodiscard]] bool is_coincident() const noexcept {
        return kind == intersection_kind::coincident;
    }
};

/**
 * @brief World-space contact with parameters of both participating entities.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct intersection3 {
    point3<REAL> point; ///< Contact point in world coordinates.
    REAL s = REAL(0); ///< Ray or spline parameter.
    REAL u = REAL(0); ///< Sphere or plane u parameter.
    REAL v = REAL(0); ///< Sphere or plane v parameter.
    REAL residual = REAL(0); ///< Absolute surface residual after refinement.
};

/**
 * @brief Complete result of one intersection query.
 *
 * For coincident entities `points` is empty because no finite list is complete.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct intersection_result3 {
    intersection_kind kind = intersection_kind::none; ///< Set classification.
    std::vector<intersection3<REAL>> points; ///< Isolated contacts sorted by s.

    /** @brief Test for any isolated or coincident contact. @return True unless kind is none. */
    [[nodiscard]] bool has_intersection() const noexcept {
        return kind != intersection_kind::none;
    }

    /** @brief Test for continuous overlap. @return True when kind is coincident. */
    [[nodiscard]] bool is_coincident() const noexcept {
        return kind == intersection_kind::coincident;
    }
};

/** @cond internal */
namespace detail {

template <std::floating_point REAL, typename FUNCTION>
[[nodiscard]] bool function_is_near_zero_everywhere(
    FUNCTION&& function,
    REAL lower,
    REAL upper,
    const numerical_settings<REAL>& settings) {
    // A coincident classification is necessarily numerical.  Reuse the user
    // sampling density, capped below to protect low-sample root searches.
    const std::size_t checks = std::max<std::size_t>(settings.sample_count, 16);
    for (std::size_t index = 0; index <= checks; ++index) {
        const REAL fraction = static_cast<REAL>(index) / static_cast<REAL>(checks);
        const REAL parameter = lower + fraction * (upper - lower);
        if (std::abs(function(parameter)) > settings.residual_tolerance) {
            return false;
        }
    }
    return true;
}

template <std::floating_point REAL>
[[nodiscard]] intersection_kind kind_for_count(std::size_t count) noexcept {
    return count == 0 ? intersection_kind::none : intersection_kind::discrete;
}

template <std::floating_point REAL, typename SPLINE>
void suppress_closed_seam_duplicate(
    std::vector<REAL>& roots,
    const SPLINE& spline,
    REAL parameter_tolerance) {
    if (!spline.is_closed() || roots.size() < 2) {
        return;
    }
    if (std::abs(roots.front() - spline.s_min()) <= parameter_tolerance &&
        std::abs(roots.back() - spline.s_max()) <= parameter_tolerance) {
        roots.pop_back();
    }
}

} // namespace detail
/** @endcond */

/**
 * @brief Numerically intersect a forward 2D ray with a 2D circle.
 *
 * All residual evaluation and root refinement occurs in the independent 2D
 * world. Root coverage depends on `settings.sample_count`.
 *
 * @tparam REAL Floating-point scalar type.
 * @param ray Forward 2D ray queried for `s >= 0`.
 * @param circle Target 2D circle.
 * @param settings Numerical tolerances and sampling controls.
 * @return Classified 2D contacts sorted by increasing s.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result2<REAL> intersect_ray_circle(
    const ray2<REAL>& ray,
    const circle2<REAL>& circle,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    const REAL direction_length = ray.direction().length();
    const REAL world_bound = distance(ray.origin(), circle.center()) +
                             circle.radius() + settings.residual_tolerance;
    const REAL s_upper = std::max(
        world_bound / direction_length,
        settings.parameter_tolerance * REAL(2));
    const auto residual = [&](REAL s) {
        return distance(ray.point_at(s), circle.center()) - circle.radius();
    };

    const std::vector<REAL> roots = find_roots<REAL>(
        residual, REAL(0), s_upper, settings);
    intersection_result2<REAL> result;
    for (REAL s : roots) {
        const point2<REAL> point = ray.point_at(s);
        const REAL angular_tolerance = std::min(
            settings.residual_tolerance, circle.radius() * REAL(0.5));
        result.points.push_back(
            {point, s, circle.parameter_of(point, angular_tolerance),
             std::abs(residual(s))});
    }
    result.kind = detail::kind_for_count<REAL>(result.points.size());
    return result;
}

/**
 * @brief Numerically intersect a 2D NURBS spline with a 2D circle.
 *
 * All residual evaluation and root refinement occurs in the independent 2D
 * world. Root coverage depends on `settings.sample_count`.
 *
 * @tparam REAL Floating-point scalar type.
 * @param spline Target 2D NURBS curve over its active s domain.
 * @param circle Target 2D circle.
 * @param settings Numerical tolerances and sampling controls.
 * @return Classified 2D contacts sorted by increasing s.
 * @throws std::domain_error When the spline repeats without a finite limit.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result2<REAL> intersect_spline_circle(
    const nurbs_spline2<REAL>& spline,
    const circle2<REAL>& circle,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    if (spline.period_count() == 0) {
        throw std::domain_error(
            "2D spline-circle intersection requires a finite spline domain");
    }
    const auto residual = [&](REAL s) {
        return distance(spline.evaluate(s), circle.center()) - circle.radius();
    };
    if (detail::function_is_near_zero_everywhere(
            residual, spline.s_min(), spline.s_max(), settings)) {
        return {intersection_kind::coincident, {}};
    }

    std::vector<REAL> roots = find_roots<REAL>(
        residual, spline.s_min(), spline.s_max(), settings);
    detail::suppress_closed_seam_duplicate(
        roots, spline, settings.parameter_tolerance);
    intersection_result2<REAL> result;
    for (REAL s : roots) {
        const point2<REAL> point = spline.evaluate(s);
        const REAL angular_tolerance = std::min(
            settings.residual_tolerance, circle.radius() * REAL(0.5));
        result.points.push_back(
            {point, s, circle.parameter_of(point, angular_tolerance),
             std::abs(residual(s))});
    }
    result.kind = detail::kind_for_count<REAL>(result.points.size());
    return result;
}

/**
 * @brief Numerically intersect a forward ray with a sphere.
 * @tparam REAL Floating-point scalar type.
 * @param ray Forward ray queried for `s >= 0`.
 * @param sphere Target sphere.
 * @param settings Numerical tolerances and sampling controls.
 * @return Classified contacts sorted by increasing s.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result3<REAL> intersect_ray_sphere(
    const ray3<REAL>& ray,
    const sphere3<REAL>& sphere,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    const REAL direction_length = ray.direction().length();
    const REAL world_bound = distance(ray.origin(), sphere.center()) +
                             sphere.radius() + settings.residual_tolerance;
    const REAL s_upper = std::max(
        world_bound / direction_length,
        settings.parameter_tolerance * REAL(2));

    const auto residual = [&](REAL s) {
        return distance(ray.point_at(s), sphere.center()) - sphere.radius();
    };
    const std::vector<REAL> roots = find_roots<REAL>(
        residual, REAL(0), s_upper, settings);

    intersection_result3<REAL> result;
    for (REAL s : roots) {
        const point3<REAL> point = ray.point_at(s);
        const REAL angular_tolerance = std::min(
            settings.residual_tolerance, sphere.radius() * REAL(0.5));
        const auto [u, v] = sphere.parameters_of(point, angular_tolerance);
        result.points.push_back({point, s, u, v, std::abs(residual(s))});
    }
    result.kind = detail::kind_for_count<REAL>(result.points.size());
    return result;
}

/**
 * @brief Numerically intersect a forward ray with an infinite plane.
 * @tparam REAL Floating-point scalar type.
 * @param ray Forward ray queried for `s >= 0`.
 * @param plane Target infinite plane.
 * @param settings Numerical tolerances and sampling controls.
 * @return None, one isolated contact, or a coincident classification.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result3<REAL> intersect_ray_plane(
    const ray3<REAL>& ray,
    const plane3<REAL>& plane,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    const REAL origin_residual = plane.signed_distance_to(ray.origin());
    const REAL rate = ray.direction().dot(plane.normal());

    if (std::abs(rate) <= settings.residual_tolerance) {
        return {
            std::abs(origin_residual) <= settings.residual_tolerance
                ? intersection_kind::coincident
                : intersection_kind::none,
            {}};
    }

    // The quotient is used only to establish a finite bracket. The contact is
    // still found by the shared numerical root solver and verified by residual.
    const REAL estimated_s = -origin_residual / rate;
    if (estimated_s < -settings.parameter_tolerance) {
        return {};
    }
    if (std::abs(estimated_s) <= settings.parameter_tolerance) {
        const auto [u, v] = plane.parameters_of(ray.origin());
        return {intersection_kind::discrete,
                {{ray.origin(), REAL(0), u, v, std::abs(origin_residual)}}};
    }

    const REAL s_upper = estimated_s * REAL(2) + settings.parameter_tolerance;
    const auto residual = [&](REAL s) {
        return plane.signed_distance_to(ray.point_at(s));
    };
    const std::vector<REAL> roots = find_roots<REAL>(
        residual, REAL(0), s_upper, settings);

    intersection_result3<REAL> result;
    for (REAL s : roots) {
        const point3<REAL> point = ray.point_at(s);
        const auto [u, v] = plane.parameters_of(point);
        result.points.push_back({point, s, u, v, std::abs(residual(s))});
    }
    result.kind = detail::kind_for_count<REAL>(result.points.size());
    return result;
}

/**
 * @brief Numerically intersect a spline with a sphere over its active domain.
 * @tparam REAL Floating-point scalar type.
 * @param spline Target NURBS curve.
 * @param sphere Target sphere.
 * @param settings Numerical tolerances and sampling controls.
 * @return Classified sampled-and-refined contacts sorted by s.
 * @throws std::domain_error When the spline repeats without a finite limit.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result3<REAL> intersect_spline_sphere(
    const nurbs_spline3<REAL>& spline,
    const sphere3<REAL>& sphere,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    if (spline.period_count() == 0) {
        throw std::domain_error(
            "spline-sphere intersection requires a finite spline domain");
    }
    const auto residual = [&](REAL s) {
        return distance(spline.evaluate(s), sphere.center()) - sphere.radius();
    };

    if (detail::function_is_near_zero_everywhere(
            residual, spline.s_min(), spline.s_max(), settings)) {
        return {intersection_kind::coincident, {}};
    }

    auto roots = find_roots<REAL>(
        residual, spline.s_min(), spline.s_max(), settings);
    detail::suppress_closed_seam_duplicate(
        roots, spline, settings.parameter_tolerance);
    intersection_result3<REAL> result;
    for (REAL s : roots) {
        const point3<REAL> point = spline.evaluate(s);
        const REAL angular_tolerance = std::min(
            settings.residual_tolerance, sphere.radius() * REAL(0.5));
        const auto [u, v] = sphere.parameters_of(point, angular_tolerance);
        result.points.push_back({point, s, u, v, std::abs(residual(s))});
    }
    result.kind = detail::kind_for_count<REAL>(result.points.size());
    return result;
}

/**
 * @brief Numerically intersect a spline with an infinite plane.
 * @tparam REAL Floating-point scalar type.
 * @param spline Target NURBS curve.
 * @param plane Target infinite plane.
 * @param settings Numerical tolerances and sampling controls.
 * @return Classified sampled-and-refined contacts sorted by s.
 * @throws std::domain_error When the spline repeats without a finite limit.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result3<REAL> intersect_spline_plane(
    const nurbs_spline3<REAL>& spline,
    const plane3<REAL>& plane,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    if (spline.period_count() == 0) {
        throw std::domain_error(
            "spline-plane intersection requires a finite spline domain");
    }
    const auto residual = [&](REAL s) {
        return plane.signed_distance_to(spline.evaluate(s));
    };

    if (detail::function_is_near_zero_everywhere(
            residual, spline.s_min(), spline.s_max(), settings)) {
        return {intersection_kind::coincident, {}};
    }

    auto roots = find_roots<REAL>(
        residual, spline.s_min(), spline.s_max(), settings);
    detail::suppress_closed_seam_duplicate(
        roots, spline, settings.parameter_tolerance);
    intersection_result3<REAL> result;
    for (REAL s : roots) {
        const point3<REAL> point = spline.evaluate(s);
        const auto [u, v] = plane.parameters_of(point);
        result.points.push_back({point, s, u, v, std::abs(residual(s))});
    }
    result.kind = detail::kind_for_count<REAL>(result.points.size());
    return result;
}

/**
 * @brief Convenience overload for ray-sphere intersection.
 * @tparam REAL Floating-point scalar type.
 * @param ray Forward ray.
 * @param sphere Target sphere.
 * @param settings Numerical controls.
 * @return Ray-sphere intersection result.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result3<REAL> intersect(
    const ray3<REAL>& ray,
    const sphere3<REAL>& sphere,
    const numerical_settings<REAL>& settings = {}) {
    return intersect_ray_sphere(ray, sphere, settings);
}

/**
 * @brief Convenience overload for ray-plane intersection.
 * @tparam REAL Floating-point scalar type.
 * @param ray Forward ray.
 * @param plane Target plane.
 * @param settings Numerical controls.
 * @return Ray-plane intersection result.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result3<REAL> intersect(
    const ray3<REAL>& ray,
    const plane3<REAL>& plane,
    const numerical_settings<REAL>& settings = {}) {
    return intersect_ray_plane(ray, plane, settings);
}

/**
 * @brief Convenience overload for spline-sphere intersection.
 * @tparam REAL Floating-point scalar type.
 * @param spline Target spline.
 * @param sphere Target sphere.
 * @param settings Numerical controls.
 * @return Spline-sphere intersection result.
 * @throws std::domain_error When the spline repeats without a finite limit.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result3<REAL> intersect(
    const nurbs_spline3<REAL>& spline,
    const sphere3<REAL>& sphere,
    const numerical_settings<REAL>& settings = {}) {
    return intersect_spline_sphere(spline, sphere, settings);
}

/**
 * @brief Convenience overload for spline-plane intersection.
 * @tparam REAL Floating-point scalar type.
 * @param spline Target spline.
 * @param plane Target plane.
 * @param settings Numerical controls.
 * @return Spline-plane intersection result.
 * @throws std::domain_error When the spline repeats without a finite limit.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result3<REAL> intersect(
    const nurbs_spline3<REAL>& spline,
    const plane3<REAL>& plane,
    const numerical_settings<REAL>& settings = {}) {
    return intersect_spline_plane(spline, plane, settings);
}

/**
 * @brief Convenience overload for 2D ray-circle intersection.
 * @tparam REAL Floating-point scalar type.
 * @param ray Forward ray in the 2D world.
 * @param circle Target circle in the same 2D world.
 * @param settings Numerical controls.
 * @return Two-dimensional ray-circle intersection result.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result2<REAL> intersect(
    const ray2<REAL>& ray,
    const circle2<REAL>& circle,
    const numerical_settings<REAL>& settings = {}) {
    return intersect_ray_circle(ray, circle, settings);
}

/**
 * @brief Convenience overload for 2D spline-circle intersection.
 * @tparam REAL Floating-point scalar type.
 * @param spline NURBS spline in the 2D world.
 * @param circle Target circle in the same 2D world.
 * @param settings Numerical controls.
 * @return Two-dimensional spline-circle intersection result.
 * @throws std::domain_error When the spline repeats without a finite limit.
 */
template <std::floating_point REAL>
[[nodiscard]] intersection_result2<REAL> intersect(
    const nurbs_spline2<REAL>& spline,
    const circle2<REAL>& circle,
    const numerical_settings<REAL>& settings = {}) {
    return intersect_spline_circle(spline, circle, settings);
}

} // namespace nurbspath
