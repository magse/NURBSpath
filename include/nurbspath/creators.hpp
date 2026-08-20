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
#include <cstddef>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>
#include <valarray>
#include <vector>

namespace nurbspath {

namespace detail {

/**
 * @brief Copy the indexed values of a valarray into ordinary contiguous storage.
 * @tparam value_type Stored value type.
 * @param values Values to copy in index order.
 * @return Vector containing the same values in the same order.
 *
 * `std::valarray` deliberately exposes no general-purpose container interface.
 * Keeping this conversion here lets the public factories share the existing
 * vector-based NURBS constructors and all of their validation.
 */
template <typename value_type>
[[nodiscard]] std::vector<value_type> valarray_to_vector(
    const std::valarray<value_type>& values) {
    std::vector<value_type> result;
    result.reserve(values.size());
    for (std::size_t index = 0; index < values.size(); ++index) {
        result.push_back(values[index]);
    }
    return result;
}

} // namespace detail

/**
 * @brief Allocate a zero 2D vector with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @return Unique smart pointer owning the new vector.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<vector2<REAL>> make_vector2() {
    return std::make_unique<vector2<REAL>>();
}

/**
 * @brief Allocate a 2D vector with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param x X component in the 2D world.
 * @param y Y component in the 2D world.
 * @return Unique smart pointer owning the new vector.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<vector2<REAL>> make_vector2(REAL x, REAL y) {
    return std::make_unique<vector2<REAL>>(vector2<REAL>{x, y});
}

/**
 * @brief Allocate the 2D origin with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @return Unique smart pointer owning the new point.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<point2<REAL>> make_point2() {
    return std::make_unique<point2<REAL>>();
}

/**
 * @brief Allocate a 2D point with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param x X coordinate in the 2D world.
 * @param y Y coordinate in the 2D world.
 * @return Unique smart pointer owning the new point.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<point2<REAL>> make_point2(REAL x, REAL y) {
    return std::make_unique<point2<REAL>>(point2<REAL>{x, y});
}

/**
 * @brief Allocate a 2D ray with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param origin Base point at `s = 0`.
 * @param direction Nonzero, unnormalized parameter direction.
 * @param tolerance Minimum accepted direction length.
 * @return Unique smart pointer owning the new ray.
 * @throws std::invalid_argument When direction is within tolerance of zero.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<ray2<REAL>> make_ray2(
    const point2<REAL>& origin,
    const vector2<REAL>& direction,
    REAL tolerance = vector2<REAL>::default_tolerance()) {
    return std::make_unique<ray2<REAL>>(origin, direction, tolerance);
}

/**
 * @brief Allocate a 2D ray defined by two points with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param origin Base point at `s = 0`.
 * @param through_point Second point, reached at `s = 1`, that defines the
 * positive ray direction.
 * @param tolerance Minimum accepted separation between the two points.
 * @return Unique smart pointer owning the new ray. Its unnormalized direction
 * is `through_point - origin`.
 * @throws std::invalid_argument When the point separation is within tolerance
 * of zero.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<ray2<REAL>> make_ray2_from_points(
    const point2<REAL>& origin,
    const point2<REAL>& through_point,
    REAL tolerance = vector2<REAL>::default_tolerance()) {
    return make_ray2<REAL>(origin, through_point - origin, tolerance);
}

/**
 * @brief Allocate a 2D circle with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param center Circle center in the 2D world.
 * @param radius Positive radius in 2D world units.
 * @return Unique smart pointer owning the new circle.
 * @throws std::invalid_argument When radius is not finite and positive.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<circle2<REAL>> make_circle2(
    const point2<REAL>& center,
    REAL radius) {
    return std::make_unique<circle2<REAL>>(center, radius);
}

/**
 * @brief Allocate a complete 2D NURBS definition with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param control_points Control points in the 2D world.
 * @param weights Positive rational weights.
 * @param knots Nondecreasing knot vector.
 * @param degree Positive degree below the control-point count.
 * @param tolerance Positive definition and parameter-boundary tolerance.
 * @return Unique smart pointer owning the new 2D spline.
 * @throws std::invalid_argument When the NURBS definition is invalid.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<nurbs_spline2<REAL>> make_nurbs_spline2(
    std::vector<point2<REAL>> control_points,
    std::vector<REAL> weights,
    std::vector<REAL> knots,
    std::size_t degree,
    REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon()) {
    return std::make_unique<nurbs_spline2<REAL>>(
        std::move(control_points),
        std::move(weights),
        std::move(knots),
        degree,
        tolerance);
}

/**
 * @brief Allocate an open or closed complete 2D NURBS definition.
 * @tparam REAL Floating-point scalar type.
 * @param control_points Control points in the 2D world.
 * @param weights Positive rational weights.
 * @param knots Nondecreasing knot vector.
 * @param degree Positive degree below the control-point count.
 * @param closed True when the active-domain endpoints must coincide.
 * @param tolerance Positive validation and parameter-boundary tolerance.
 * @return Unique smart pointer owning the new 2D spline.
 * @throws std::invalid_argument When the definition or closed seam is invalid.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<nurbs_spline2<REAL>> make_nurbs_spline2(
    std::vector<point2<REAL>> control_points,
    std::vector<REAL> weights,
    std::vector<REAL> knots,
    std::size_t degree,
    bool closed,
    REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon()) {
    return std::make_unique<nurbs_spline2<REAL>>(
        std::move(control_points),
        std::move(weights),
        std::move(knots),
        degree,
        closed,
        tolerance);
}

/**
 * @brief Allocate a complete 2D NURBS definition from valarrays.
 * @tparam REAL Floating-point scalar type.
 * @tparam control_point_array Deduced `std::valarray<point2<REAL>>` type.
 * @param control_points Control points copied in 2D valarray index order.
 * @param weights Positive rational weights copied in valarray index order.
 * @param knots Nondecreasing knots copied in valarray index order.
 * @param degree Positive degree below the control-point count.
 * @param tolerance Positive definition and parameter-boundary tolerance.
 * @return Unique smart pointer owning the new 2D spline.
 * @throws std::invalid_argument When the NURBS definition is invalid.
 *
 * The deduced control-point parameter accepts const, mutable, lvalue, and
 * rvalue valarrays. Its constraint keeps brace-initializer calls unambiguous
 * with the existing vector overload, while weights and knots deduce `REAL`.
 */
template <
    std::floating_point REAL,
    typename control_point_array>
requires
    std::same_as<
        std::remove_cvref_t<control_point_array>,
        std::valarray<point2<REAL>>>
[[nodiscard]] std::unique_ptr<nurbs_spline2<REAL>> make_nurbs_spline2(
    control_point_array&& control_points,
    const std::valarray<REAL>& weights,
    const std::valarray<REAL>& knots,
    std::size_t degree,
    REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon()) {
    return make_nurbs_spline2<REAL>(
        detail::valarray_to_vector(control_points),
        detail::valarray_to_vector(weights),
        detail::valarray_to_vector(knots),
        degree,
        tolerance);
}

/**
 * @brief Allocate an open or closed 2D NURBS definition from valarrays.
 * @tparam REAL Floating-point scalar type.
 * @tparam control_point_array Deduced `std::valarray<point2<REAL>>` type.
 * @param control_points Control points copied in valarray index order.
 * @param weights Positive rational weights copied in valarray index order.
 * @param knots Nondecreasing knots copied in valarray index order.
 * @param degree Positive degree below the control-point count.
 * @param closed True when the active-domain endpoints must coincide.
 * @param tolerance Positive validation and parameter-boundary tolerance.
 * @return Unique smart pointer owning the new 2D spline.
 * @throws std::invalid_argument When the definition or closed seam is invalid.
 */
template <
    std::floating_point REAL,
    typename control_point_array>
requires
    std::same_as<
        std::remove_cvref_t<control_point_array>,
        std::valarray<point2<REAL>>>
[[nodiscard]] std::unique_ptr<nurbs_spline2<REAL>> make_nurbs_spline2(
    control_point_array&& control_points,
    const std::valarray<REAL>& weights,
    const std::valarray<REAL>& knots,
    std::size_t degree,
    bool closed,
    REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon()) {
    return make_nurbs_spline2<REAL>(
        detail::valarray_to_vector(control_points),
        detail::valarray_to_vector(weights),
        detail::valarray_to_vector(knots),
        degree,
        closed,
        tolerance);
}

/**
 * @brief Allocate a zero 3D vector with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @return Unique smart pointer owning the new vector.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<vector3<REAL>> make_vector3() {
    return std::make_unique<vector3<REAL>>();
}

/**
 * @brief Allocate a 3D vector with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param x X component in the 3D world.
 * @param y Y component in the 3D world.
 * @param z Z component in the 3D world.
 * @return Unique smart pointer owning the new vector.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<vector3<REAL>> make_vector3(
    REAL x,
    REAL y,
    REAL z) {
    return std::make_unique<vector3<REAL>>(vector3<REAL>{x, y, z});
}

/**
 * @brief Allocate the 3D world origin with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @return Unique smart pointer owning the new point.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<point3<REAL>> make_point3() {
    return std::make_unique<point3<REAL>>();
}

/**
 * @brief Allocate a 3D point with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param x X coordinate in the 3D world.
 * @param y Y coordinate in the 3D world.
 * @param z Z coordinate in the 3D world.
 * @return Unique smart pointer owning the new point.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<point3<REAL>> make_point3(
    REAL x,
    REAL y,
    REAL z) {
    return std::make_unique<point3<REAL>>(point3<REAL>{x, y, z});
}

/**
 * @brief Allocate a 3D ray with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param origin World-space base point at `s = 0`.
 * @param direction Nonzero, unnormalized parameter direction.
 * @param tolerance Minimum accepted direction length.
 * @return Unique smart pointer owning the new ray.
 * @throws std::invalid_argument When direction is within tolerance of zero.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<ray3<REAL>> make_ray3(
    const point3<REAL>& origin,
    const vector3<REAL>& direction,
    REAL tolerance = vector3<REAL>::default_tolerance()) {
    return std::make_unique<ray3<REAL>>(origin, direction, tolerance);
}

/**
 * @brief Allocate a 3D ray defined by two world points with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param origin World-space base point at `s = 0`.
 * @param through_point Second world point, reached at `s = 1`, that defines the
 * positive ray direction.
 * @param tolerance Minimum accepted separation between the two points.
 * @return Unique smart pointer owning the new ray. Its unnormalized direction
 * is `through_point - origin`.
 * @throws std::invalid_argument When the point separation is within tolerance
 * of zero.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<ray3<REAL>> make_ray3_from_points(
    const point3<REAL>& origin,
    const point3<REAL>& through_point,
    REAL tolerance = vector3<REAL>::default_tolerance()) {
    return make_ray3<REAL>(origin, through_point - origin, tolerance);
}

/**
 * @brief Allocate a 3D sphere with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param center World-space sphere center.
 * @param radius Positive radius in world units.
 * @return Unique smart pointer owning the new sphere.
 * @throws std::invalid_argument When radius is not positive.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<sphere3<REAL>> make_sphere3(
    const point3<REAL>& center,
    REAL radius) {
    return std::make_unique<sphere3<REAL>>(center, radius);
}

/**
 * @brief Allocate a Hessian normal-form plane with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param normal Any nonzero plane normal; normalized internally.
 * @param signed_distance Signed perpendicular distance from the world origin.
 * @param tolerance Minimum accepted normal length.
 * @return Unique smart pointer owning the new plane.
 * @throws std::invalid_argument When distance is non-finite.
 * @throws std::domain_error When normal is too small.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<plane3<REAL>> make_plane3(
    const vector3<REAL>& normal,
    REAL signed_distance,
    REAL tolerance = vector3<REAL>::default_tolerance()) {
    return std::make_unique<plane3<REAL>>(normal, signed_distance, tolerance);
}

/**
 * @brief Allocate a point-and-normal plane with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param origin World-space point used as `(u,v) = (0,0)`.
 * @param normal Any nonzero plane normal; normalized internally.
 * @param tolerance Minimum accepted normal length.
 * @return Unique smart pointer owning the new plane.
 * @throws std::domain_error When normal is too small.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<plane3<REAL>> make_plane3(
    const point3<REAL>& origin,
    const vector3<REAL>& normal,
    REAL tolerance = vector3<REAL>::default_tolerance()) {
    return std::make_unique<plane3<REAL>>(origin, normal, tolerance);
}

/**
 * @brief Allocate a plane with a preferred positive-u direction.
 * @tparam REAL Floating-point scalar type.
 * @param origin World-space point used as `(u,v) = (0,0)`.
 * @param normal Any nonzero plane normal; normalized internally.
 * @param u_hint Direction projected into the plane to define positive u.
 * @param tolerance Minimum accepted normal and projected-u length.
 * @return Unique smart pointer owning the new plane.
 * @throws std::domain_error When normal or projected u hint is too small.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<plane3<REAL>> make_plane3(
    const point3<REAL>& origin,
    const vector3<REAL>& normal,
    const vector3<REAL>& u_hint,
    REAL tolerance = vector3<REAL>::default_tolerance()) {
    return std::make_unique<plane3<REAL>>(origin, normal, u_hint, tolerance);
}

/**
 * @brief Allocate a plane from a parameter origin, positive-u direction, and
 * another plane point with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param origin World-space point used as `(u,v) = (0,0)`.
 * @param u_direction Any nonzero direction defining positive u; normalized
 * internally.
 * @param plane_point World-space point whose component perpendicular to the
 * u-axis defines the positive-v side of the plane.
 * @param tolerance Minimum accepted u-direction length and perpendicular
 * distance from `plane_point` to the u-axis.
 * @return Unique smart pointer owning a plane with a right-handed
 * `(u,v,normal)` frame.
 * @throws std::domain_error When the u direction is too small or the plane
 * point lies within tolerance of the u-axis.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<plane3<REAL>> make_plane3_from_u_direction(
    const point3<REAL>& origin,
    const vector3<REAL>& u_direction,
    const point3<REAL>& plane_point,
    REAL tolerance = vector3<REAL>::default_tolerance()) {
    const vector3<REAL> unit_u = u_direction.normalized(tolerance);
    const vector3<REAL> plane_offset = plane_point - origin;
    const vector3<REAL> unit_v =
        (plane_offset - plane_offset.dot(unit_u) * unit_u).normalized(tolerance);
    const vector3<REAL> normal = unit_u.cross(unit_v);
    return std::make_unique<plane3<REAL>>(origin, normal, unit_u);
}

/**
 * @brief Allocate a plane from three non-collinear world points with unique
 * ownership.
 * @tparam REAL Floating-point scalar type.
 * @param origin World-space point used as `(u,v) = (0,0)`.
 * @param u_point World-space point for which `u_point - origin` defines the
 * positive-u direction.
 * @param plane_point World-space point whose component perpendicular to the
 * u-axis defines the positive-v side of the plane.
 * @param tolerance Minimum accepted separation from `origin` to `u_point` and
 * perpendicular distance from `plane_point` to the u-axis.
 * @return Unique smart pointer owning a plane with a right-handed
 * `(u,v,normal)` frame.
 * @throws std::domain_error When the first two points are within tolerance or
 * all three points are collinear within tolerance.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<plane3<REAL>> make_plane3_from_points(
    const point3<REAL>& origin,
    const point3<REAL>& u_point,
    const point3<REAL>& plane_point,
    REAL tolerance = vector3<REAL>::default_tolerance()) {
    return make_plane3_from_u_direction<REAL>(
        origin, u_point - origin, plane_point, tolerance);
}

/**
 * @brief Allocate a complete 3D NURBS definition with unique ownership.
 * @tparam REAL Floating-point scalar type.
 * @param control_points World-space control points.
 * @param weights Positive rational weights.
 * @param knots Nondecreasing knot vector.
 * @param degree Positive degree below the control-point count.
 * @param tolerance Positive definition and parameter-boundary tolerance.
 * @return Unique smart pointer owning the new 3D spline.
 * @throws std::invalid_argument When the NURBS definition is invalid.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<nurbs_spline3<REAL>> make_nurbs_spline3(
    std::vector<point3<REAL>> control_points,
    std::vector<REAL> weights,
    std::vector<REAL> knots,
    std::size_t degree,
    REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon()) {
    return std::make_unique<nurbs_spline3<REAL>>(
        std::move(control_points),
        std::move(weights),
        std::move(knots),
        degree,
        tolerance);
}

/**
 * @brief Allocate an open or closed complete 3D NURBS definition.
 * @tparam REAL Floating-point scalar type.
 * @param control_points World-space control points.
 * @param weights Positive rational weights.
 * @param knots Nondecreasing knot vector.
 * @param degree Positive degree below the control-point count.
 * @param closed True when the active-domain endpoints must coincide.
 * @param tolerance Positive validation and parameter-boundary tolerance.
 * @return Unique smart pointer owning the new 3D spline.
 * @throws std::invalid_argument When the definition or closed seam is invalid.
 */
template <std::floating_point REAL>
[[nodiscard]] std::unique_ptr<nurbs_spline3<REAL>> make_nurbs_spline3(
    std::vector<point3<REAL>> control_points,
    std::vector<REAL> weights,
    std::vector<REAL> knots,
    std::size_t degree,
    bool closed,
    REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon()) {
    return std::make_unique<nurbs_spline3<REAL>>(
        std::move(control_points),
        std::move(weights),
        std::move(knots),
        degree,
        closed,
        tolerance);
}

/**
 * @brief Allocate a complete 3D NURBS definition from valarrays.
 * @tparam REAL Floating-point scalar type.
 * @tparam control_point_array Deduced `std::valarray<point3<REAL>>` type.
 * @param control_points World-space control points copied in valarray order.
 * @param weights Positive rational weights copied in valarray index order.
 * @param knots Nondecreasing knots copied in valarray index order.
 * @param degree Positive degree below the control-point count.
 * @param tolerance Positive definition and parameter-boundary tolerance.
 * @return Unique smart pointer owning the new 3D spline.
 * @throws std::invalid_argument When the NURBS definition is invalid.
 *
 * The deduced control-point parameter accepts const, mutable, lvalue, and
 * rvalue valarrays. Its constraint keeps brace-initializer calls unambiguous
 * with the existing vector overload, while weights and knots deduce `REAL`.
 */
template <
    std::floating_point REAL,
    typename control_point_array>
requires
    std::same_as<
        std::remove_cvref_t<control_point_array>,
        std::valarray<point3<REAL>>>
[[nodiscard]] std::unique_ptr<nurbs_spline3<REAL>> make_nurbs_spline3(
    control_point_array&& control_points,
    const std::valarray<REAL>& weights,
    const std::valarray<REAL>& knots,
    std::size_t degree,
    REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon()) {
    return make_nurbs_spline3<REAL>(
        detail::valarray_to_vector(control_points),
        detail::valarray_to_vector(weights),
        detail::valarray_to_vector(knots),
        degree,
        tolerance);
}

/**
 * @brief Allocate an open or closed 3D NURBS definition from valarrays.
 * @tparam REAL Floating-point scalar type.
 * @tparam control_point_array Deduced `std::valarray<point3<REAL>>` type.
 * @param control_points World-space control points copied in valarray order.
 * @param weights Positive rational weights copied in valarray index order.
 * @param knots Nondecreasing knots copied in valarray index order.
 * @param degree Positive degree below the control-point count.
 * @param closed True when the active-domain endpoints must coincide.
 * @param tolerance Positive validation and parameter-boundary tolerance.
 * @return Unique smart pointer owning the new 3D spline.
 * @throws std::invalid_argument When the definition or closed seam is invalid.
 */
template <
    std::floating_point REAL,
    typename control_point_array>
requires
    std::same_as<
        std::remove_cvref_t<control_point_array>,
        std::valarray<point3<REAL>>>
[[nodiscard]] std::unique_ptr<nurbs_spline3<REAL>> make_nurbs_spline3(
    control_point_array&& control_points,
    const std::valarray<REAL>& weights,
    const std::valarray<REAL>& knots,
    std::size_t degree,
    bool closed,
    REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon()) {
    return make_nurbs_spline3<REAL>(
        detail::valarray_to_vector(control_points),
        detail::valarray_to_vector(weights),
        detail::valarray_to_vector(knots),
        degree,
        closed,
        tolerance);
}

} // namespace nurbspath
