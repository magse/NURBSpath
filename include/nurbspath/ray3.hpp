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
        if (direction_.is_near_zero(tolerance)) {
            throw std::invalid_argument("ray3 direction must be non-zero");
        }
    }

    /** @brief Get the base point. @return Constant reference to the origin. */
    [[nodiscard]] const point3<REAL>& origin() const noexcept { return origin_; }
    /** @brief Get the unnormalized parameter direction. @return Ray direction. */
    [[nodiscard]] const vector3<REAL>& direction() const noexcept { return direction_; }

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
    point3<REAL> origin_;
    vector3<REAL> direction_;
};

} // namespace nurbspath
