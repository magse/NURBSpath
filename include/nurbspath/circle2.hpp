#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point2.hpp"

#include <cmath>
#include <concepts>
#include <numbers>
#include <stdexcept>

namespace nurbspath {

/**
 * @brief Circle in the independent two-dimensional Cartesian world.
 *
 * The angular parameter `u` is measured counterclockwise from positive X and
 * is reported in `[0,2*pi)`.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class circle2 {
public:
    /**
     * @brief Construct a circle.
     * @param center_value Center in the 2D world.
     * @param radius_value Positive radius in 2D world units.
     * @throws std::invalid_argument When radius is not finite and positive.
     */
    circle2(const point2<REAL>& center_value, REAL radius_value)
        : center_(center_value), radius_(radius_value) {
        if (!(radius_ > REAL(0)) || !std::isfinite(radius_)) {
            throw std::invalid_argument("circle2 radius must be finite and positive");
        }
    }

    /** @brief Get the center. @return Constant center reference. */
    [[nodiscard]] const point2<REAL>& center() const noexcept { return center_; }
    /** @brief Get the radius. @return Radius in 2D world units. */
    [[nodiscard]] REAL radius() const noexcept { return radius_; }

    /**
     * @brief Evaluate the circle parameterization.
     * @param u Counterclockwise angle in radians.
     * @return Point on the circle in the 2D world.
     */
    [[nodiscard]] point2<REAL> point_at(REAL u) const noexcept {
        return center_ + radius_ * vector2<REAL>{std::cos(u), std::sin(u)};
    }

    /**
     * @brief Compute the outward radial unit normal at a point.
     * @param point Noncentral point, normally on the circle.
     * @return Outward unit direction after radial projection.
     * @throws std::domain_error When point equals the center.
     */
    [[nodiscard]] vector2<REAL> normal_at(const point2<REAL>& point) const {
        return (point - center_).normalized(REAL(0));
    }

    /**
     * @brief Recover the angular parameter by radial projection.
     * @param point Noncentral point in the 2D world.
     * @param tolerance Nonnegative center-distance rejection tolerance.
     * @return Counterclockwise angle in `[0,2*pi)`.
     * @throws std::invalid_argument When tolerance is negative.
     * @throws std::domain_error When point is within tolerance of the center.
     */
    [[nodiscard]] REAL parameter_of(
        const point2<REAL>& point,
        REAL tolerance = REAL(0)) const {
        if (tolerance < REAL(0)) {
            throw std::invalid_argument("circle parameter tolerance cannot be negative");
        }
        const vector2<REAL> radial = point - center_;
        if (radial.length() <= tolerance) {
            throw std::domain_error("circle parameter is undefined at its center");
        }
        REAL u = std::atan2(radial.y(), radial.x());
        if (u < REAL(0)) {
            u += REAL(2) * std::numbers::pi_v<REAL>;
        }
        return u;
    }

private:
    point2<REAL> center_;
    REAL radius_;
};

} // namespace nurbspath
