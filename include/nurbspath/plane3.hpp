#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point3.hpp"

#include <cmath>
#include <concepts>
#include <stdexcept>
#include <utility>

namespace nurbspath {

/**
 * @brief Infinite plane with an orthonormal `(u,v)` coordinate frame.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class plane3 {
public:
    /**
     * @brief Construct a plane in Hessian normal form.
     *
     * The resulting equation is `unit_normal dot world_point = signed_distance`.
     * Its `(u,v)` origin is the closest plane point to the world origin.
     *
     * @param normal_value Any nonzero plane normal; normalized internally.
     * @param signed_distance Signed perpendicular distance from world origin.
     * @param tolerance Minimum accepted normal length.
     * @throws std::invalid_argument When distance is non-finite.
     * @throws std::domain_error When normal is too small.
     */
    plane3(
        const vector3<REAL>& normal_value,
        REAL signed_distance,
        REAL tolerance = vector3<REAL>::default_tolerance())
        : plane3(
              point_from_normal_and_distance(
                  normal_value, signed_distance, tolerance),
              normal_value,
              tolerance) {}

    /**
     * @brief Construct a plane from a parameter origin and normal.
     * @param origin_value World-space point used as `(u,v) = (0,0)`.
     * @param normal_value Any nonzero plane normal; normalized internally.
     * @param tolerance Minimum accepted normal length.
     * @throws std::domain_error When normal is too small.
     */
    plane3(
        const point3<REAL>& origin_value,
        const vector3<REAL>& normal_value,
        REAL tolerance = vector3<REAL>::default_tolerance())
        : origin_(origin_value), normal_(normal_value.normalized(tolerance)) {
        // Pick the Cartesian axis least aligned with the normal.  Its cross
        // product gives a stable first in-plane basis direction.
        const vector3<REAL> helper =
            std::abs(normal_.x()) <= std::abs(normal_.y()) &&
                    std::abs(normal_.x()) <= std::abs(normal_.z())
                ? vector3<REAL>::unit_x()
                : (std::abs(normal_.y()) <= std::abs(normal_.z())
                       ? vector3<REAL>::unit_y()
                       : vector3<REAL>::unit_z());
        u_direction_ = helper.cross(normal_).normalized(tolerance);
        v_direction_ = normal_.cross(u_direction_);
    }

    /**
     * @brief Construct a plane with a preferred positive-u direction.
     * @param origin_value World-space point used as `(u,v) = (0,0)`.
     * @param normal_value Any nonzero plane normal; normalized internally.
     * @param u_hint Direction projected into the plane to define positive u.
     * @param tolerance Minimum accepted normal and projected-u length.
     * @throws std::domain_error When normal or projected u hint is too small.
     */
    plane3(
        const point3<REAL>& origin_value,
        const vector3<REAL>& normal_value,
        const vector3<REAL>& u_hint,
        REAL tolerance = vector3<REAL>::default_tolerance())
        : origin_(origin_value), normal_(normal_value.normalized(tolerance)) {
        // Remove the normal component so the caller's hint lies in the plane.
        u_direction_ = u_hint.rejected_from(normal_, tolerance).normalized(tolerance);
        v_direction_ = normal_.cross(u_direction_);
    }

    /** @brief Get the plane parameter origin. @return Constant origin reference. */
    [[nodiscard]] const point3<REAL>& origin() const noexcept { return origin_; }
    /** @brief Get the unit normal. @return Constant normal reference. */
    [[nodiscard]] const vector3<REAL>& normal() const noexcept { return normal_; }
    /** @brief Get the positive unit-u direction. @return Constant u direction reference. */
    [[nodiscard]] const vector3<REAL>& u_direction() const noexcept { return u_direction_; }
    /** @brief Get the positive unit-v direction. @return Constant v direction reference. */
    [[nodiscard]] const vector3<REAL>& v_direction() const noexcept { return v_direction_; }

    /**
     * @brief Get the equivalent Hessian signed distance.
     * @return `d` for the equation `unit_normal dot x = d`.
     */
    [[nodiscard]] constexpr REAL signed_distance_from_origin() const noexcept {
        return (origin_ - point3<REAL>::origin()).dot(normal_);
    }

    /**
     * @brief Evaluate plane parameters.
     * @param u Coordinate along positive u direction.
     * @param v Coordinate along positive v direction.
     * @return World-space plane point.
     */
    [[nodiscard]] constexpr point3<REAL> point_at(REAL u, REAL v) const noexcept {
        return origin_ + u * u_direction_ + v * v_direction_;
    }

    /**
     * @brief Compute oriented perpendicular distance to the plane.
     * @param point World-space query point.
     * @return Signed distance, positive in the normal direction.
     */
    [[nodiscard]] constexpr REAL signed_distance_to(
        const point3<REAL>& point) const noexcept {
        return (point - origin_).dot(normal_);
    }

    /**
     * @brief Orthogonally project a point onto the plane.
     * @param point World-space query point.
     * @return Closest point on the plane.
     */
    [[nodiscard]] constexpr point3<REAL> project(
        const point3<REAL>& point) const noexcept {
        return point - signed_distance_to(point) * normal_;
    }

    /**
     * @brief Recover plane coordinates after orthogonal projection.
     * @param point World-space query point.
     * @return Pair `(u,v)` in the plane frame.
     */
    [[nodiscard]] constexpr std::pair<REAL, REAL> parameters_of(
        const point3<REAL>& point) const noexcept {
        const vector3<REAL> offset = point - origin_;
        return {offset.dot(u_direction_), offset.dot(v_direction_)};
    }

private:
    [[nodiscard]] static point3<REAL> point_from_normal_and_distance(
        const vector3<REAL>& normal,
        REAL signed_distance,
        REAL tolerance) {
        if (!std::isfinite(signed_distance)) {
            throw std::invalid_argument("plane signed distance must be finite");
        }
        return point3<REAL>::origin() +
               signed_distance * normal.normalized(tolerance);
    }

    point3<REAL> origin_;
    vector3<REAL> normal_;
    vector3<REAL> u_direction_;
    vector3<REAL> v_direction_;
};

} // namespace nurbspath
