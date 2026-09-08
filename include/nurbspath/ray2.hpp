#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point2.hpp"

#include <concepts>
#include <stdexcept>

namespace nurbspath {

/**
 * @brief 2D half-line parameterized as `origin + s * direction` for `s >= 0`.
 *
 * The direction is not normalized. The ray exists only in the 2D world until
 * explicitly embedded with `project(plane, ray)`.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class ray2 {
public:
    /**
     * @brief Construct a forward 2D ray.
     * @param origin_value 2D base point at `s = 0`.
     * @param direction_value Nonzero 2D parameter direction.
     * @param tolerance Minimum accepted direction length.
     * @throws std::invalid_argument When direction is within tolerance of zero.
     */
    ray2(
        const point2<REAL>& origin_value,
        const vector2<REAL>& direction_value,
        REAL tolerance = vector2<REAL>::default_tolerance())
        : origin_(origin_value), direction_(direction_value) {
        validate_direction(direction_, tolerance);
    }

    /** @brief Get the base point. @return Constant origin reference. */
    [[nodiscard]] const point2<REAL>& origin() const noexcept { return origin_; }
    /** @brief Get the unnormalized direction. @return Constant direction reference. */
    [[nodiscard]] const vector2<REAL>& direction() const noexcept { return direction_; }

    /**
     * @brief Get the base point.
     * @return Constant reference to the origin in the 2D world.
     */
    [[nodiscard]] const point2<REAL>& get_origin() const noexcept {
        return origin_;
    }

    /**
     * @brief Get the unnormalized direction.
     * @return Constant reference to the parameter direction.
     */
    [[nodiscard]] const vector2<REAL>& get_direction() const noexcept {
        return direction_;
    }

    /**
     * @brief Replace the origin while preserving the direction.
     * @param origin_value New origin in the 2D world.
     */
    void set_origin(const point2<REAL>& origin_value) noexcept {
        origin_ = origin_value;
    }

    /**
     * @brief Replace the origin using vector components while preserving direction.
     * @tparam VECTOR Exactly `vector2<REAL>`; the template form keeps
     * brace-list calls to the point overload unambiguous.
     * @param origin_value Vector whose components become the origin coordinates.
     */
    template <typename VECTOR>
        requires std::same_as<VECTOR, vector2<REAL>>
    void set_origin(const VECTOR& origin_value) noexcept {
        origin_ = origin_value;
    }

    /**
     * @brief Replace the unnormalized parameter direction.
     * @param direction_value New nonzero direction.
     * @param tolerance Minimum accepted direction length.
     * @throws std::invalid_argument When direction is within tolerance of zero.
     * The ray remains unchanged when validation fails.
     */
    void set_direction(
        const vector2<REAL>& direction_value,
        REAL tolerance = vector2<REAL>::default_tolerance()) {
        validate_direction(direction_value, tolerance);
        direction_ = direction_value;
    }

    /**
     * @brief Replace the origin and direction together.
     * @param origin_value New origin in the 2D world.
     * @param direction_value New nonzero, unnormalized direction.
     * @param tolerance Minimum accepted direction length.
     * @throws std::invalid_argument When direction is within tolerance of zero.
     * The ray remains unchanged when validation fails.
     */
    void set_origin_and_direction(
        const point2<REAL>& origin_value,
        const vector2<REAL>& direction_value,
        REAL tolerance = vector2<REAL>::default_tolerance()) {
        validate_direction(direction_value, tolerance);
        origin_ = origin_value;
        direction_ = direction_value;
    }

    /**
     * @brief Replace the origin from vector components and replace the direction.
     * @tparam VECTOR Exactly `vector2<REAL>`; the template form keeps
     * brace-list calls to the point overload unambiguous.
     * @param origin_value Vector whose components become the origin coordinates.
     * @param direction_value New nonzero, unnormalized direction.
     * @param tolerance Minimum accepted direction length.
     * @throws std::invalid_argument When direction is within tolerance of zero.
     * The ray remains unchanged when validation fails.
     */
    template <typename VECTOR>
        requires std::same_as<VECTOR, vector2<REAL>>
    void set_origin_and_direction(
        const VECTOR& origin_value,
        const vector2<REAL>& direction_value,
        REAL tolerance = vector2<REAL>::default_tolerance()) {
        validate_direction(direction_value, tolerance);
        origin_ = origin_value;
        direction_ = direction_value;
    }

    /**
     * @brief Evaluate the ray parameterization.
     * @param s Native ray parameter coordinate.
     * @return `origin + s * direction` in the 2D world.
     */
    [[nodiscard]] constexpr point2<REAL> point_at(REAL s) const noexcept {
        return origin_ + s * direction_;
    }

    /**
     * @brief Evaluate the ray parameterization.
     * @param s Native ray parameter coordinate.
     * @return `origin + s * direction` in the 2D world.
     */
    [[nodiscard]] constexpr point2<REAL> evaluate(REAL s) const noexcept {
        return point_at(s);
    }

    /** @brief Get the unit positive parameter direction. @return Normalized direction. */
    [[nodiscard]] vector2<REAL> tangent() const { return direction_.normalized(); }

private:
    static void validate_direction(
        const vector2<REAL>& direction_value,
        REAL tolerance) {
        if (direction_value.is_near_zero(tolerance)) {
            throw std::invalid_argument("ray2 direction must be non-zero");
        }
    }

    point2<REAL> origin_;
    vector2<REAL> direction_;
};

template <std::floating_point REAL>
inline vector2<REAL>::vector2(const ray2<REAL>& ray_value) noexcept
    : vector2(ray_value.origin()) {}

} // namespace nurbspath
