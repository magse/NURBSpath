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
        validate_radius(radius_);
    }

    /** @brief Get the center. @return Constant center reference. */
    [[nodiscard]] const point2<REAL>& center() const noexcept { return center_; }
    /** @brief Get the radius. @return Radius in 2D world units. */
    [[nodiscard]] REAL radius() const noexcept { return radius_; }

    /**
     * @brief Get the center.
     * @return Constant reference to the center in the 2D world.
     */
    [[nodiscard]] const point2<REAL>& get_center() const noexcept {
        return center_;
    }

    /**
     * @brief Get the radius.
     * @return Radius in 2D world units.
     */
    [[nodiscard]] REAL get_radius() const noexcept { return radius_; }

    /**
     * @brief Replace the center while preserving the radius.
     * @param center_value New center in the 2D world.
     */
    void set_center(const point2<REAL>& center_value) noexcept {
        center_ = center_value;
    }

    /**
     * @brief Replace the center using vector components while preserving radius.
     * @tparam VECTOR Exactly `vector2<REAL>`; the template form keeps
     * brace-list calls to the point overload unambiguous.
     * @param center_value Vector whose components become the center coordinates.
     */
    template <typename VECTOR>
        requires std::same_as<VECTOR, vector2<REAL>>
    void set_center(const VECTOR& center_value) noexcept {
        center_ = center_value;
    }

    /**
     * @brief Replace the radius while preserving the center.
     * @param radius_value New positive, finite radius in 2D world units.
     * @throws std::invalid_argument When radius is not finite and positive.
     */
    void set_radius(REAL radius_value) {
        validate_radius(radius_value);
        radius_ = radius_value;
    }

    /**
     * @brief Replace the center and radius together.
     * @param center_value New center in the 2D world.
     * @param radius_value New positive, finite radius in 2D world units.
     * @throws std::invalid_argument When radius is not finite and positive.
     * The circle remains unchanged when validation fails.
     */
    void set_center_and_radius(
        const point2<REAL>& center_value,
        REAL radius_value) {
        validate_radius(radius_value);
        center_ = center_value;
        radius_ = radius_value;
    }

    /**
     * @brief Replace the center from vector components and replace the radius.
     * @tparam VECTOR Exactly `vector2<REAL>`; the template form keeps
     * brace-list calls to the point overload unambiguous.
     * @param center_value Vector whose components become the center coordinates.
     * @param radius_value New positive, finite radius in 2D world units.
     * @throws std::invalid_argument When radius is not finite and positive.
     * The circle remains unchanged when validation fails.
     */
    template <typename VECTOR>
        requires std::same_as<VECTOR, vector2<REAL>>
    void set_center_and_radius(
        const VECTOR& center_value,
        REAL radius_value) {
        validate_radius(radius_value);
        center_ = center_value;
        radius_ = radius_value;
    }

    /**
     * @brief Replace the center using vector components while preserving the radius.
     * @param center_value Vector whose components become the center coordinates.
     * @return Reference to this circle.
     */
    circle2& operator=(const vector2<REAL>& center_value) noexcept {
        center_ = center_value;
        return *this;
    }

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
        REAL u = std::atan2(radial.y, radial.x);
        if (u < REAL(0)) {
            u += REAL(2) * std::numbers::pi_v<REAL>;
        }
        return u;
    }

private:
    static void validate_radius(REAL radius_value) {
        if (!(radius_value > REAL(0)) || !std::isfinite(radius_value)) {
            throw std::invalid_argument(
                "circle2 radius must be finite and positive");
        }
    }

    point2<REAL> center_;
    REAL radius_;
};

template <std::floating_point REAL>
inline vector2<REAL>::vector2(const circle2<REAL>& circle_value) noexcept
    : vector2(circle_value.center()) {}

} // namespace nurbspath
