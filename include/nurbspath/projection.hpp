#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/circle2.hpp"
#include "nurbspath/nurbs_spline2.hpp"
#include "nurbspath/nurbs_spline3.hpp"
#include "nurbspath/plane3.hpp"
#include "nurbspath/ray2.hpp"
#include "nurbspath/ray3.hpp"
#include "nurbspath/sphere3.hpp"

#include <concepts>
#include <vector>

namespace nurbspath {

/**
 * @brief Embed a 2D vector in a plane's orthonormal parameter frame.
 * @tparam REAL Floating-point scalar type.
 * @param plane Target 3D plane defining the u and v basis directions.
 * @param vector Vector in the independent 2D world.
 * @return `vector.x*u_direction + vector.y*v_direction` in 3D.
 */
template <std::floating_point REAL>
[[nodiscard]] constexpr vector3<REAL> project(
    const plane3<REAL>& plane,
    const vector2<REAL>& vector) noexcept {
    return vector.x() * plane.u_direction() + vector.y() * plane.v_direction();
}

/**
 * @brief Embed a 2D point in a plane's parameter frame.
 * @tparam REAL Floating-point scalar type.
 * @param plane Target 3D plane whose origin represents the 2D origin.
 * @param point Point in the independent 2D world.
 * @return 3D point `plane.point_at(point.x, point.y)`.
 */
template <std::floating_point REAL>
[[nodiscard]] constexpr point3<REAL> project(
    const plane3<REAL>& plane,
    const point2<REAL>& point) noexcept {
    return plane.point_at(point.x(), point.y());
}

/**
 * @brief Embed a 2D ray in a plane's parameter frame.
 * @tparam REAL Floating-point scalar type.
 * @param plane Target 3D plane.
 * @param ray Ray in the independent 2D world.
 * @return 3D ray with projected origin and direction and the same native s.
 */
template <std::floating_point REAL>
[[nodiscard]] ray3<REAL> project(
    const plane3<REAL>& plane,
    const ray2<REAL>& ray) {
    return ray3<REAL>{project(plane, ray.origin()), project(plane, ray.direction())};
}

/**
 * @brief Convert a 2D circle's center and radius to a 3D sphere.
 *
 * Only center and radius are carried across worlds. The result is a complete
 * sphere centered at the projected 2D center; it is not a planar circle in 3D.
 *
 * @tparam REAL Floating-point scalar type.
 * @param plane Target 3D plane used to project the circle center.
 * @param circle Circle in the independent 2D world.
 * @return Sphere with projected center and unchanged radius.
 */
template <std::floating_point REAL>
[[nodiscard]] sphere3<REAL> project(
    const plane3<REAL>& plane,
    const circle2<REAL>& circle) {
    return sphere3<REAL>{project(plane, circle.center()), circle.radius()};
}

/**
 * @brief Embed a 2D NURBS spline by projecting its control points.
 *
 * Weights, knots, degree, closure state, and native `s` domain are copied
 * unchanged. Because the plane mapping is affine, evaluating the resulting 3D
 * spline is equivalent to projecting the corresponding evaluated 2D point.
 *
 * @tparam REAL Floating-point scalar type.
 * @param plane Target 3D plane.
 * @param spline NURBS spline in the independent 2D world.
 * @return 3D NURBS spline with projected control points.
 */
template <std::floating_point REAL>
[[nodiscard]] nurbs_spline3<REAL> project(
    const plane3<REAL>& plane,
    const nurbs_spline2<REAL>& spline) {
    std::vector<point3<REAL>> control_points;
    control_points.reserve(spline.control_points().size());
    for (const point2<REAL>& control_point : spline.control_points()) {
        control_points.push_back(project(plane, control_point));
    }
    return nurbs_spline3<REAL>{
        std::move(control_points),
        spline.weights(),
        spline.knots(),
        spline.degree(),
        spline.is_closed(),
        spline.tolerance()};
}

} // namespace nurbspath
