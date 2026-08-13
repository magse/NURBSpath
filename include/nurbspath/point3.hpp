#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/vector3.hpp"

#include <concepts>
#include <cstddef>
#include <stdexcept>

namespace nurbspath {

/**
 * @brief Position in the shared Cartesian world coordinate system.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class point3 {
public:
    using value_type = REAL; ///< Floating-point scalar type.

    /** @brief Construct the world origin. */
    constexpr point3() noexcept = default;

    /**
     * @brief Construct a point from Cartesian coordinates.
     * @param x_value X coordinate.
     * @param y_value Y coordinate.
     * @param z_value Z coordinate.
     */
    constexpr point3(REAL x_value, REAL y_value, REAL z_value) noexcept
        : x_(x_value), y_(y_value), z_(z_value) {}

    /**
     * @brief Compare coordinates exactly.
     * @param other Point to compare.
     * @return True when all coordinates are exactly equal.
     */
    [[nodiscard]] constexpr bool operator==(const point3& other) const noexcept = default;

    /** @brief Get the X coordinate. @return X coordinate. */
    [[nodiscard]] constexpr REAL x() const noexcept { return x_; }
    /** @brief Get the Y coordinate. @return Y coordinate. */
    [[nodiscard]] constexpr REAL y() const noexcept { return y_; }
    /** @brief Get the Z coordinate. @return Z coordinate. */
    [[nodiscard]] constexpr REAL z() const noexcept { return z_; }

    /** @brief Set the X coordinate. @param value New X coordinate. */
    constexpr void set_x(REAL value) noexcept { x_ = value; }
    /** @brief Set the Y coordinate. @param value New Y coordinate. */
    constexpr void set_y(REAL value) noexcept { y_ = value; }
    /** @brief Set the Z coordinate. @param value New Z coordinate. */
    constexpr void set_z(REAL value) noexcept { z_ = value; }

    /**
     * @brief Access a coordinate by index.
     * @param index Coordinate index: 0 for X, 1 for Y, or 2 for Z.
     * @return Mutable reference to the selected coordinate.
     * @throws std::out_of_range When index is greater than 2.
     */
    [[nodiscard]] constexpr REAL& operator[](std::size_t index) {
        if (index > 2) {
            throw std::out_of_range("point3 index must be 0, 1, or 2");
        }
        return index == 0 ? x_ : (index == 1 ? y_ : z_);
    }

    /**
     * @brief Access a coordinate by index.
     * @param index Coordinate index: 0 for X, 1 for Y, or 2 for Z.
     * @return Constant reference to the selected coordinate.
     * @throws std::out_of_range When index is greater than 2.
     */
    [[nodiscard]] constexpr const REAL& operator[](std::size_t index) const {
        if (index > 2) {
            throw std::out_of_range("point3 index must be 0, 1, or 2");
        }
        return index == 0 ? x_ : (index == 1 ? y_ : z_);
    }

    /**
     * @brief Translate this point in place.
     * @param offset World-space translation vector.
     * @return Reference to this point.
     */
    constexpr point3& operator+=(const vector3<REAL>& offset) noexcept {
        x_ += offset.x();
        y_ += offset.y();
        z_ += offset.z();
        return *this;
    }

    /**
     * @brief Translate this point by the negative of a vector.
     * @param offset World-space translation vector to subtract.
     * @return Reference to this point.
     */
    constexpr point3& operator-=(const vector3<REAL>& offset) noexcept {
        x_ -= offset.x();
        y_ -= offset.y();
        z_ -= offset.z();
        return *this;
    }

    /**
     * @brief Compare points using Euclidean distance.
     * @param other Point to compare.
     * @param tolerance Maximum accepted distance.
     * @return True when positions differ by at most tolerance.
     */
    [[nodiscard]] bool approximately_equal(
        const point3& other,
        REAL tolerance = vector3<REAL>::default_tolerance()) const noexcept {
        return (*this - other).length_squared() <= tolerance * tolerance;
    }

    /** @brief Construct the Cartesian world origin. @return (0,0,0). */
    [[nodiscard]] static constexpr point3 origin() noexcept { return {}; }

private:
    REAL x_ = REAL(0);
    REAL y_ = REAL(0);
    REAL z_ = REAL(0);
};

template <std::floating_point REAL>
/**
 * @brief Translate a point by a vector.
 * @param point Point to translate.
 * @param offset Translation vector.
 * @return Translated point.
 */
[[nodiscard]] constexpr point3<REAL> operator+(
    point3<REAL> point,
    const vector3<REAL>& offset) noexcept {
    point += offset;
    return point;
}

template <std::floating_point REAL>
/**
 * @brief Translate a point by a vector with vector-first syntax.
 * @param offset Translation vector.
 * @param point Point to translate.
 * @return Translated point.
 */
[[nodiscard]] constexpr point3<REAL> operator+(
    const vector3<REAL>& offset,
    point3<REAL> point) noexcept {
    point += offset;
    return point;
}

template <std::floating_point REAL>
/**
 * @brief Translate a point by the negative of a vector.
 * @param point Point to translate.
 * @param offset Translation vector to subtract.
 * @return Translated point.
 */
[[nodiscard]] constexpr point3<REAL> operator-(
    point3<REAL> point,
    const vector3<REAL>& offset) noexcept {
    point -= offset;
    return point;
}

template <std::floating_point REAL>
/**
 * @brief Form the displacement from one point to another.
 * @param left Destination point.
 * @param right Source point.
 * @return Vector from right to left.
 */
[[nodiscard]] constexpr vector3<REAL> operator-(
    const point3<REAL>& left,
    const point3<REAL>& right) noexcept {
    return {left.x() - right.x(), left.y() - right.y(), left.z() - right.z()};
}

template <std::floating_point REAL>
/**
 * @brief Linearly interpolate two points.
 * @param first Position at fraction zero.
 * @param second Position at fraction one.
 * @param fraction Interpolation fraction; extrapolation is permitted.
 * @return Interpolated position.
 */
[[nodiscard]] constexpr point3<REAL> lerp(
    const point3<REAL>& first,
    const point3<REAL>& second,
    REAL fraction) noexcept {
    return first + fraction * (second - first);
}

template <std::floating_point REAL>
/**
 * @brief Compute squared Euclidean distance between points.
 * @param first First point.
 * @param second Second point.
 * @return Squared distance.
 */
[[nodiscard]] REAL distance_squared(
    const point3<REAL>& first,
    const point3<REAL>& second) noexcept {
    return (first - second).length_squared();
}

template <std::floating_point REAL>
/**
 * @brief Compute Euclidean distance between points.
 * @param first First point.
 * @param second Second point.
 * @return Distance.
 */
[[nodiscard]] REAL distance(
    const point3<REAL>& first,
    const point3<REAL>& second) noexcept {
    return (first - second).length();
}

} // namespace nurbspath
