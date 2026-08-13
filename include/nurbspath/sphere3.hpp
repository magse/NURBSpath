#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point3.hpp"

#include <cmath>
#include <concepts>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace nurbspath {

/**
 * @brief Sphere in world coordinates.
 *
 * `u` is longitude in `[0, 2*pi)` and `v` is latitude in
 * `[-pi/2, pi/2]`.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class sphere3 {
public:
    /**
     * @brief Construct a sphere.
     * @param center_value World-space center.
     * @param radius_value Positive radius in world units.
     * @throws std::invalid_argument When radius is not positive.
     */
    sphere3(const point3<REAL>& center_value, REAL radius_value)
        : center_(center_value), radius_(radius_value) {
        if (!(radius_ > REAL(0))) {
            throw std::invalid_argument("sphere3 radius must be positive");
        }
    }

    /** @brief Get the world-space center. @return Constant center reference. */
    [[nodiscard]] const point3<REAL>& center() const noexcept { return center_; }
    /** @brief Get the radius. @return Radius in world units. */
    [[nodiscard]] REAL radius() const noexcept { return radius_; }

    /**
     * @brief Evaluate spherical surface parameters.
     * @param u Longitude in radians.
     * @param v Latitude in radians.
     * @return World-space surface point.
     */
    [[nodiscard]] point3<REAL> point_at(REAL u, REAL v) const noexcept {
        const REAL cos_v = std::cos(v);
        return center_ + radius_ * vector3<REAL>{
            cos_v * std::cos(u),
            cos_v * std::sin(u),
            std::sin(v)
        };
    }

    /**
     * @brief Compute the outward radial unit normal at a point.
     * @param point Noncentral point, normally on the sphere surface.
     * @return Outward unit normal after radial projection.
     * @throws std::domain_error When point equals the sphere center.
     */
    [[nodiscard]] vector3<REAL> normal_at(const point3<REAL>& point) const {
        // Radius scale must not affect whether a valid surface normal exists.
        // A zero tolerance rejects only the exactly central, undefined case.
        return (point - center_).normalized(REAL(0));
    }

    /**
     * @brief Recover surface coordinates by radial projection.
     * @param point Noncentral world-space point.
     * @param tolerance Nonnegative center-distance rejection tolerance.
     * @return Pair `(u,v)` containing longitude and latitude in radians.
     * @throws std::invalid_argument When tolerance is negative.
     * @throws std::domain_error When point is within tolerance of the center.
     */
    [[nodiscard]] std::pair<REAL, REAL> parameters_of(
        const point3<REAL>& point,
        REAL tolerance = REAL(0)) const {
        if (tolerance < REAL(0)) {
            throw std::invalid_argument("sphere parameter tolerance cannot be negative");
        }
        const vector3<REAL> radial = point - center_;
        const REAL magnitude = radial.length();
        if (magnitude <= tolerance) {
            throw std::domain_error("sphere parameters are undefined at its center");
        }

        REAL u = std::atan2(radial.y(), radial.x());
        if (u < REAL(0)) {
            u += REAL(2) * std::numbers::pi_v<REAL>;
        }
        const REAL v = std::asin(std::clamp(radial.z() / magnitude, REAL(-1), REAL(1)));
        return {u, v};
    }

private:
    point3<REAL> center_;
    REAL radius_;
};

} // namespace nurbspath
