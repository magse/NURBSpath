#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/vector2.hpp"

#include <concepts>
#include <cstddef>
#include <stdexcept>

namespace nurbspath {

/**
 * @brief Position in the independent two-dimensional Cartesian world.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class point2 {
public:
    using value_type = REAL; ///< Floating-point scalar type.

    /** @brief Construct the 2D origin. */
    constexpr point2() noexcept = default;

    /**
     * @brief Construct a point from Cartesian coordinates.
     * @param x_value X coordinate in the 2D world.
     * @param y_value Y coordinate in the 2D world.
     */
    constexpr point2(REAL x_value, REAL y_value) noexcept
        : x_(x_value), y_(y_value) {}

    /**
     * @brief Compare coordinates exactly.
     * @param other Point to compare.
     * @return True when both coordinates are exactly equal.
     */
    [[nodiscard]] constexpr bool operator==(const point2& other) const noexcept = default;

    /** @brief Get the X coordinate. @return X coordinate. */
    [[nodiscard]] constexpr REAL x() const noexcept { return x_; }
    /** @brief Get the Y coordinate. @return Y coordinate. */
    [[nodiscard]] constexpr REAL y() const noexcept { return y_; }

    /** @brief Set the X coordinate. @param value New X coordinate. */
    constexpr void set_x(REAL value) noexcept { x_ = value; }
    /** @brief Set the Y coordinate. @param value New Y coordinate. */
    constexpr void set_y(REAL value) noexcept { y_ = value; }

    /**
     * @brief Access a coordinate by index.
     * @param index Coordinate index: 0 for X or 1 for Y.
     * @return Mutable reference to the selected coordinate.
     * @throws std::out_of_range When index is greater than 1.
     */
    [[nodiscard]] constexpr REAL& operator[](std::size_t index) {
        if (index > 1) {
            throw std::out_of_range("point2 index must be 0 or 1");
        }
        return index == 0 ? x_ : y_;
    }

    /**
     * @brief Access a coordinate by index.
     * @param index Coordinate index: 0 for X or 1 for Y.
     * @return Constant reference to the selected coordinate.
     * @throws std::out_of_range When index is greater than 1.
     */
    [[nodiscard]] constexpr const REAL& operator[](std::size_t index) const {
        if (index > 1) {
            throw std::out_of_range("point2 index must be 0 or 1");
        }
        return index == 0 ? x_ : y_;
    }

    /**
     * @brief Translate this point in place.
     * @param offset Translation vector in the 2D world.
     * @return Reference to this point.
     */
    constexpr point2& operator+=(const vector2<REAL>& offset) noexcept {
        x_ += offset.x();
        y_ += offset.y();
        return *this;
    }

    /**
     * @brief Translate this point by the negative of a vector.
     * @param offset Translation vector to subtract.
     * @return Reference to this point.
     */
    constexpr point2& operator-=(const vector2<REAL>& offset) noexcept {
        x_ -= offset.x();
        y_ -= offset.y();
        return *this;
    }

    /**
     * @brief Compare points using Euclidean distance.
     * @param other Point to compare.
     * @param tolerance Maximum accepted 2D distance.
     * @return True when positions differ by at most tolerance.
     */
    [[nodiscard]] bool approximately_equal(
        const point2& other,
        REAL tolerance = vector2<REAL>::default_tolerance()) const noexcept {
        return (*this - other).length_squared() <= tolerance * tolerance;
    }

    /** @brief Construct the Cartesian 2D origin. @return `(0,0)`. */
    [[nodiscard]] static constexpr point2 origin() noexcept { return {}; }

private:
    REAL x_ = REAL(0);
    REAL y_ = REAL(0);
};

/** @brief Translate a point by a vector. @param point Point to translate. @param offset Translation vector. @return Translated point. */
template <std::floating_point REAL>
[[nodiscard]] constexpr point2<REAL> operator+(
    point2<REAL> point,
    const vector2<REAL>& offset) noexcept {
    point += offset;
    return point;
}

/** @brief Translate a point with vector-first syntax. @param offset Translation vector. @param point Point to translate. @return Translated point. */
template <std::floating_point REAL>
[[nodiscard]] constexpr point2<REAL> operator+(
    const vector2<REAL>& offset,
    point2<REAL> point) noexcept {
    point += offset;
    return point;
}

/** @brief Translate a point by the negative of a vector. @param point Point to translate. @param offset Translation vector to subtract. @return Translated point. */
template <std::floating_point REAL>
[[nodiscard]] constexpr point2<REAL> operator-(
    point2<REAL> point,
    const vector2<REAL>& offset) noexcept {
    point -= offset;
    return point;
}

/** @brief Form the displacement from one point to another. @param left Destination point. @param right Source point. @return Vector from right to left. */
template <std::floating_point REAL>
[[nodiscard]] constexpr vector2<REAL> operator-(
    const point2<REAL>& left,
    const point2<REAL>& right) noexcept {
    return {left.x() - right.x(), left.y() - right.y()};
}

/** @brief Linearly interpolate points. @param first Position at fraction zero. @param second Position at fraction one. @param fraction Interpolation fraction. @return Interpolated position. */
template <std::floating_point REAL>
[[nodiscard]] constexpr point2<REAL> lerp(
    const point2<REAL>& first,
    const point2<REAL>& second,
    REAL fraction) noexcept {
    return first + fraction * (second - first);
}

/** @brief Compute squared Euclidean point distance. @param first First point. @param second Second point. @return Squared distance. */
template <std::floating_point REAL>
[[nodiscard]] REAL distance_squared(
    const point2<REAL>& first,
    const point2<REAL>& second) noexcept {
    return (first - second).length_squared();
}

/** @brief Compute Euclidean point distance. @param first First point. @param second Second point. @return Distance. */
template <std::floating_point REAL>
[[nodiscard]] REAL distance(
    const point2<REAL>& first,
    const point2<REAL>& second) noexcept {
    return (first - second).length();
}

} // namespace nurbspath
