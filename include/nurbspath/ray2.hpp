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
        if (direction_.is_near_zero(tolerance)) {
            throw std::invalid_argument("ray2 direction must be non-zero");
        }
    }

    /** @brief Get the base point. @return Constant origin reference. */
    [[nodiscard]] const point2<REAL>& origin() const noexcept { return origin_; }
    /** @brief Get the unnormalized direction. @return Constant direction reference. */
    [[nodiscard]] const vector2<REAL>& direction() const noexcept { return direction_; }

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
    point2<REAL> origin_;
    vector2<REAL> direction_;
};

} // namespace nurbspath
