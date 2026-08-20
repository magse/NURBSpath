#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/serialization.hpp"
#include "nurbspath/vector2.hpp"

#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <istream>
#include <ostream>
#include <stdexcept>

namespace nurbspath {

/**
 * @brief Position in the independent two-dimensional Cartesian world.
 *
 * Coordinates are public, zero-initialized aggregate members.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct point2 {
    using value_type = REAL; ///< Floating-point scalar type.

    REAL x = REAL(0); ///< X coordinate in the 2D world.
    REAL y = REAL(0); ///< Y coordinate in the 2D world.

    /**
     * @brief Compare coordinates exactly.
     * @param other Point to compare.
     * @return True when both coordinates are exactly equal.
     */
    [[nodiscard]] constexpr bool operator==(const point2& other) const noexcept = default;

    /**
     * @brief Write the coordinates in CSV, TSV, or whitespace-delimited text.
     * @param output Destination text stream.
     * @param format Delimited text format; CSV is the default.
     * @param decimal_places Nonnegative number of digits after the decimal point;
     * six by default.
     * @return Reference to output.
     * @throws std::invalid_argument When format is unsupported or
     * decimal_places is negative.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& csv_write(
        std::ostream& output,
        text_format format = text_format::csv,
        std::streamsize decimal_places = 6) const {
        return detail::write_text_coordinates(
            output, std::array<REAL, 2>{x, y}, format, decimal_places);
    }

    /**
     * @brief Read the coordinates from CSV, TSV, or whitespace-delimited text.
     * @param input Source text stream.
     * @param format Delimited text format; CSV is the default.
     * @return Reference to input. This point changes only after a complete record.
     * @throws std::invalid_argument When format is unsupported.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::istream& csv_read(
        std::istream& input,
        text_format format = text_format::csv) {
        std::array<REAL, 2> coordinates{};
        detail::read_text_coordinates(input, coordinates, format);
        if (input) {
            *this = {coordinates[0], coordinates[1]};
        }
        return input;
    }

    /**
     * @brief Write two native `REAL` values in X/Y order as binary data.
     * @param output Destination binary stream.
     * @return Reference to output.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& write(std::ostream& output) const {
        return detail::write_binary_coordinates(
            output, std::array<REAL, 2>{x, y});
    }

    /**
     * @brief Read two native `REAL` values in X/Y order from binary data.
     * @param input Source binary stream.
     * @return Reference to input. This point changes only after a complete record.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::istream& read(std::istream& input) {
        std::array<REAL, 2> coordinates{};
        detail::read_binary_coordinates(input, coordinates);
        if (input) {
            *this = {coordinates[0], coordinates[1]};
        }
        return input;
    }

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
        return index == 0 ? x : y;
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
        return index == 0 ? x : y;
    }

    /**
     * @brief Compute distance from the Cartesian 2D origin.
     * @return Euclidean magnitude in the coordinate units.
     */
    [[nodiscard]] REAL magnitude() const noexcept {
        return std::hypot(x, y);
    }

    /**
     * @brief Compute Manhattan distance from the Cartesian 2D origin.
     * @return Sum of the absolute coordinate values in the coordinate units.
     */
    [[nodiscard]] REAL manhattan_distance() const noexcept {
        return std::abs(x) + std::abs(y);
    }

    /**
     * @brief Translate this point in place.
     * @param offset Translation vector in the 2D world.
     * @return Reference to this point.
     */
    constexpr point2& operator+=(const vector2<REAL>& offset) noexcept {
        x += offset.x;
        y += offset.y;
        return *this;
    }

    /**
     * @brief Translate this point by the negative of a vector.
     * @param offset Translation vector to subtract.
     * @return Reference to this point.
     */
    constexpr point2& operator-=(const vector2<REAL>& offset) noexcept {
        x -= offset.x;
        y -= offset.y;
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
};

/**
 * @brief Write a 2D point as whitespace-separated X and Y coordinates.
 * @tparam REAL Floating-point scalar type.
 * @param output Destination stream.
 * @param value Point to write.
 * @return Reference to output.
 * @throws std::ios_base::failure When enabled by the stream exception mask.
 */
template <std::floating_point REAL>
std::ostream& operator<<(std::ostream& output, const point2<REAL>& value) {
    return output << value.x << ' ' << value.y;
}

/**
 * @brief Read whitespace-separated X and Y coordinates into a 2D point.
 * @tparam REAL Floating-point scalar type.
 * @param input Source stream.
 * @param value Point updated only when both coordinates are read successfully.
 * @return Reference to input.
 * @throws std::ios_base::failure When enabled by the stream exception mask.
 */
template <std::floating_point REAL>
std::istream& operator>>(std::istream& input, point2<REAL>& value) {
    return value.csv_read(input, text_format::txt);
}

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
    return {left.x - right.x, left.y - right.y};
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
