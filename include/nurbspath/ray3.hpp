#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point3.hpp"

#include <concepts>
#include <stdexcept>

namespace nurbspath {

/**
 * @brief Half-line parameterized as `origin + s * direction` for `s >= 0`.
 *
 * The direction is not normalized. Consequently, `s` is a world-space distance
 * only when the supplied direction is a unit vector.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class ray3 {
public:
    /**
     * @brief Construct a forward parametric ray.
     * @param origin_value World-space base point at `s = 0`.
     * @param direction_value Nonzero parameter direction.
     * @param tolerance Minimum accepted direction length.
     * @throws std::invalid_argument When direction is within tolerance of zero.
     */
    ray3(
        const point3<REAL>& origin_value,
        const vector3<REAL>& direction_value,
        REAL tolerance = vector3<REAL>::default_tolerance())
        : origin_(origin_value), direction_(direction_value) {
        validate_direction(direction_, tolerance);
    }

    /** @brief Get the base point. @return Constant reference to the origin. */
    [[nodiscard]] const point3<REAL>& origin() const noexcept { return origin_; }
    /** @brief Get the unnormalized parameter direction. @return Ray direction. */
    [[nodiscard]] const vector3<REAL>& direction() const noexcept { return direction_; }

    /**
     * @brief Get the base point.
     * @return Constant reference to the origin.
     */
    [[nodiscard]] const point3<REAL>& get_origin() const noexcept {
        return origin_;
    }

    /**
     * @brief Get the unnormalized parameter direction.
     * @return Constant reference to the ray direction.
     */
    [[nodiscard]] const vector3<REAL>& get_direction() const noexcept {
        return direction_;
    }

    /**
     * @brief Replace the base point while preserving the direction.
     * @param origin_value New world-space base point at `s = 0`.
     */
    void set_origin(const point3<REAL>& origin_value) noexcept {
        origin_ = origin_value;
    }

    /**
     * @brief Replace the base point using vector components.
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
     * @brief Replace the unnormalized parameter direction.
     * @param direction_value New nonzero parameter direction.
     * @param tolerance Minimum accepted direction length.
     * @throws std::invalid_argument When direction is within tolerance of zero.
     * The ray remains unchanged when validation fails.
     */
    void set_direction(
        const vector3<REAL>& direction_value,
        REAL tolerance = vector3<REAL>::default_tolerance()) {
        validate_direction(direction_value, tolerance);
        direction_ = direction_value;
    }

    /**
     * @brief Atomically replace the base point and parameter direction.
     * @param origin_value New world-space base point at `s = 0`.
     * @param direction_value New nonzero, unnormalized parameter direction.
     * @param tolerance Minimum accepted direction length.
     * @throws std::invalid_argument When direction is within tolerance of zero.
     * The ray remains unchanged when validation fails.
     */
    void set_origin_and_direction(
        const point3<REAL>& origin_value,
        const vector3<REAL>& direction_value,
        REAL tolerance = vector3<REAL>::default_tolerance()) {
        validate_direction(direction_value, tolerance);
        origin_ = origin_value;
        direction_ = direction_value;
    }

    /**
     * @brief Atomically replace the base point from vector components and the
     * parameter direction.
     * @tparam VECTOR Exactly `vector3<REAL>`; the template form keeps
     * brace-list calls to the point overload unambiguous.
     * @param origin_value Vector whose components become the origin coordinates.
     * @param direction_value New nonzero, unnormalized parameter direction.
     * @param tolerance Minimum accepted direction length.
     * @throws std::invalid_argument When direction is within tolerance of zero.
     * The ray remains unchanged when validation fails.
     */
    template <typename VECTOR>
        requires std::same_as<VECTOR, vector3<REAL>>
    void set_origin_and_direction(
        const VECTOR& origin_value,
        const vector3<REAL>& direction_value,
        REAL tolerance = vector3<REAL>::default_tolerance()) {
        validate_direction(direction_value, tolerance);
        origin_ = origin_value;
        direction_ = direction_value;
    }

    /**
     * @brief Evaluate the ray parameterization.
     * @param s Native ray parameter coordinate.
     * @return `origin + s * direction`.
     */
    [[nodiscard]] constexpr point3<REAL> point_at(REAL s) const noexcept {
        return origin_ + s * direction_;
    }

    /**
     * @brief Evaluate the ray parameterization.
     * @param s Native ray parameter coordinate.
     * @return `origin + s * direction`.
     */
    [[nodiscard]] constexpr point3<REAL> evaluate(REAL s) const noexcept {
        return point_at(s);
    }

    /**
     * @brief Get the unit positive parameter direction.
     * @return Normalized ray direction.
     */
    [[nodiscard]] vector3<REAL> tangent() const {
        return direction_.normalized();
    }

private:
    static void validate_direction(
        const vector3<REAL>& direction_value,
        REAL tolerance) {
        if (direction_value.is_near_zero(tolerance)) {
            throw std::invalid_argument("ray3 direction must be non-zero");
        }
    }

    point3<REAL> origin_;
    vector3<REAL> direction_;
};

template <std::floating_point REAL>
vector3<REAL>::vector3(const ray3<REAL>& ray_value) noexcept
    : x(ray_value.origin().x),
      y(ray_value.origin().y),
      z(ray_value.origin().z) {}

} // namespace nurbspath
