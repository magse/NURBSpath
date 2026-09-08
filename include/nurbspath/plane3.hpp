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
     * @param normal Any nonzero plane normal; normalized internally.
     * @param signed_distance Signed perpendicular distance from world origin.
     * @param tolerance Minimum accepted normal length.
     * @throws std::invalid_argument When distance is non-finite.
     * @throws std::domain_error When normal is too small.
     */
    plane3(
        const vector3<REAL>& normal,
        REAL signed_distance,
        REAL tolerance = vector3<REAL>::default_tolerance())
        : plane3(
              point_from_normal_and_distance(
                  normal, signed_distance, tolerance),
              normal,
              tolerance) {}

    /**
     * @brief Construct a plane from a parameter origin and normal.
     * @param origin World-space point used as `(u,v) = (0,0)`.
     * @param normal Any nonzero plane normal; normalized internally.
     * @param tolerance Minimum accepted normal length.
     * @throws std::domain_error When normal is too small.
     */
    plane3(
        const point3<REAL>& origin,
        const vector3<REAL>& normal,
        REAL tolerance = vector3<REAL>::default_tolerance())
        : origin_(origin), normal_(normal.normalized(tolerance)) {
        // Pick the Cartesian axis least aligned with the normal.  Its cross
        // product gives a stable first in-plane basis direction.
        const vector3<REAL> helper =
            std::abs(normal_.x) <= std::abs(normal_.y) &&
                    std::abs(normal_.x) <= std::abs(normal_.z)
                ? vector3<REAL>::unit_x()
                : (std::abs(normal_.y) <= std::abs(normal_.z)
                       ? vector3<REAL>::unit_y()
                       : vector3<REAL>::unit_z());
        u_direction_ = helper.cross(normal_).normalized(tolerance);
        v_direction_ = normal_.cross(u_direction_);
    }

    /**
     * @brief Construct a plane with a preferred positive-u direction.
     * @param origin World-space point used as `(u,v) = (0,0)`.
     * @param normal Any nonzero plane normal; normalized internally.
     * @param u_hint Direction projected into the plane to define positive u.
     * @param tolerance Minimum accepted normal and projected-u length.
     * @throws std::domain_error When normal or projected u hint is too small.
     */
    plane3(
        const point3<REAL>& origin,
        const vector3<REAL>& normal,
        const vector3<REAL>& u_hint,
        REAL tolerance = vector3<REAL>::default_tolerance())
        : origin_(origin), normal_(normal.normalized(tolerance)) {
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
     * @brief Get the plane parameter origin.
     * @return Constant world-space origin reference.
     */
    [[nodiscard]] const point3<REAL>& get_origin() const noexcept {
        return origin_;
    }

    /**
     * @brief Get the unit normal.
     * @return Constant normal reference.
     */
    [[nodiscard]] const vector3<REAL>& get_normal() const noexcept {
        return normal_;
    }

    /**
     * @brief Get the positive unit-u direction.
     * @return Constant u-direction reference.
     */
    [[nodiscard]] const vector3<REAL>& get_u_direction() const noexcept {
        return u_direction_;
    }

    /**
     * @brief Get the positive unit-v direction.
     * @return Constant v-direction reference.
     */
    [[nodiscard]] const vector3<REAL>& get_v_direction() const noexcept {
        return v_direction_;
    }

    /**
     * @brief Replace the parameter origin while preserving the frame directions.
     * @param origin_value New world-space parameter origin.
     */
    void set_origin(const point3<REAL>& origin_value) noexcept {
        origin_ = origin_value;
    }

    /**
     * @brief Replace the parameter origin using vector components.
     * @tparam VECTOR Exactly `vector3<REAL>`; the template form keeps
     * brace-list calls to the point overload unambiguous.
     * @param origin_value Vector whose components become the origin coordinates.
     */
    template <typename VECTOR>
        requires std::same_as<VECTOR, vector3<REAL>>
    void set_origin(const VECTOR& origin_value) noexcept {
        origin_ = origin_value;
    }

    /**
     * @brief Replace the normal and generate a stable orthonormal plane frame.
     *
     * The parameter origin is preserved. Positive u and v are regenerated in
     * the same way as by the origin-and-normal constructor.
     *
     * @param normal_value New nonzero plane normal; normalized internally.
     * @param tolerance Minimum accepted normal length.
     * @throws std::domain_error When the normal is too small. The plane is
     * unchanged when validation fails.
     */
    void set_normal(
        const vector3<REAL>& normal_value,
        REAL tolerance = vector3<REAL>::default_tolerance()) {
        const plane3 candidate(origin_, normal_value, tolerance);
        assign_definition(candidate);
    }

    /**
     * @brief Replace positive u while preserving the origin and normal.
     *
     * The supplied direction is projected into the plane and normalized;
     * positive v is regenerated to retain a right-handed orthonormal frame.
     *
     * @param u_hint Direction projected into the plane to define positive u.
     * @param tolerance Minimum accepted projected-u length.
     * @throws std::domain_error When the projected u hint is too small. The
     * plane is unchanged when validation fails.
     */
    void set_u_direction(
        const vector3<REAL>& u_hint,
        REAL tolerance = vector3<REAL>::default_tolerance()) {
        const plane3 candidate(origin_, normal_, u_hint, tolerance);
        assign_definition(candidate);
    }

    /**
     * @brief Atomically replace the plane from Hessian normal form.
     * @param normal_value New nonzero normal; normalized internally.
     * @param signed_distance New signed perpendicular distance from world origin.
     * @param tolerance Minimum accepted normal length.
     * @throws std::invalid_argument When distance is non-finite.
     * @throws std::domain_error When the normal is too small. The plane is
     * unchanged when validation fails.
     */
    void set_definition(
        const vector3<REAL>& normal_value,
        REAL signed_distance,
        REAL tolerance = vector3<REAL>::default_tolerance()) {
        const plane3 candidate(normal_value, signed_distance, tolerance);
        assign_definition(candidate);
    }

    /**
     * @brief Atomically replace the origin, normal, and generated plane frame.
     * @param origin_value New world-space parameter origin.
     * @param normal_value New nonzero plane normal; normalized internally.
     * @param tolerance Minimum accepted normal length.
     * @throws std::domain_error When the normal is too small. The plane is
     * unchanged when validation fails.
     */
    void set_definition(
        const point3<REAL>& origin_value,
        const vector3<REAL>& normal_value,
        REAL tolerance = vector3<REAL>::default_tolerance()) {
        const plane3 candidate(origin_value, normal_value, tolerance);
        assign_definition(candidate);
    }

    /**
     * @brief Atomically replace the origin from vector components and the frame.
     * @tparam VECTOR Exactly `vector3<REAL>`; the template form keeps
     * brace-list calls to the point overload unambiguous.
     * @param origin_value Vector whose components become the origin coordinates.
     * @param normal_value New nonzero plane normal; normalized internally.
     * @param tolerance Minimum accepted normal length.
     * @throws std::domain_error When the normal is too small. The plane is
     * unchanged when validation fails.
     */
    template <typename VECTOR>
        requires std::same_as<VECTOR, vector3<REAL>>
    void set_definition(
        const VECTOR& origin_value,
        const vector3<REAL>& normal_value,
        REAL tolerance = vector3<REAL>::default_tolerance()) {
        set_definition(
            point3<REAL>{origin_value.x, origin_value.y, origin_value.z},
            normal_value,
            tolerance);
    }

    /**
     * @brief Atomically replace the complete oriented plane definition.
     * @param origin_value New world-space parameter origin.
     * @param normal_value New nonzero plane normal; normalized internally.
     * @param u_hint Direction projected into the plane to define positive u.
     * @param tolerance Minimum accepted normal and projected-u length.
     * @throws std::domain_error When the normal or projected u hint is too
     * small. The plane is unchanged when validation fails.
     */
    void set_definition(
        const point3<REAL>& origin_value,
        const vector3<REAL>& normal_value,
        const vector3<REAL>& u_hint,
        REAL tolerance = vector3<REAL>::default_tolerance()) {
        const plane3 candidate(origin_value, normal_value, u_hint, tolerance);
        assign_definition(candidate);
    }

    /**
     * @brief Atomically replace an oriented plane using a vector origin.
     * @tparam VECTOR Exactly `vector3<REAL>`; the template form keeps
     * brace-list calls to the point overload unambiguous.
     * @param origin_value Vector whose components become the origin coordinates.
     * @param normal_value New nonzero plane normal; normalized internally.
     * @param u_hint Direction projected into the plane to define positive u.
     * @param tolerance Minimum accepted normal and projected-u length.
     * @throws std::domain_error When the normal or projected u hint is too
     * small. The plane is unchanged when validation fails.
     */
    template <typename VECTOR>
        requires std::same_as<VECTOR, vector3<REAL>>
    void set_definition(
        const VECTOR& origin_value,
        const vector3<REAL>& normal_value,
        const vector3<REAL>& u_hint,
        REAL tolerance = vector3<REAL>::default_tolerance()) {
        set_definition(
            point3<REAL>{origin_value.x, origin_value.y, origin_value.z},
            normal_value,
            u_hint,
            tolerance);
    }

    /**
     * @brief Get the equivalent Hessian signed distance.
     * @return `d` for the equation `unit_normal dot x = d`.
     */
    [[nodiscard]] constexpr REAL signed_distance_from_origin() const noexcept {
        return (origin_ - point3<REAL>::origin()).dot(normal_);
    }

    /**
     * @brief Get the equivalent Hessian signed distance.
     * @return `d` for the equation `unit_normal dot x = d`.
     */
    [[nodiscard]] constexpr REAL get_signed_distance_from_origin() const noexcept {
        return signed_distance_from_origin();
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
    void assign_definition(const plane3& candidate) noexcept {
        origin_ = candidate.origin_;
        normal_ = candidate.normal_;
        u_direction_ = candidate.u_direction_;
        v_direction_ = candidate.v_direction_;
    }

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

template <std::floating_point REAL>
vector3<REAL>::vector3(const plane3<REAL>& plane_value) noexcept
    : x(plane_value.origin().x),
      y(plane_value.origin().y),
      z(plane_value.origin().z) {}

} // namespace nurbspath
