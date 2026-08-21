#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/serialization.hpp"
#include "nurbspath/vector3.hpp"

#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <istream>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>

namespace nurbspath {

/**
 * @brief Position in the shared Cartesian world coordinate system.
 *
 * Coordinates are public, zero-initialized aggregate members.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct point3 {
    using value_type = REAL; ///< Floating-point scalar type.

    REAL x = REAL(0); ///< X coordinate in the 3D world.
    REAL y = REAL(0); ///< Y coordinate in the 3D world.
    REAL z = REAL(0); ///< Z coordinate in the 3D world.

    /**
     * @brief Compare coordinates exactly.
     * @param other Point to compare.
     * @return True when all coordinates are exactly equal.
     */
    [[nodiscard]] constexpr bool operator==(const point3& other) const noexcept = default;

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
            output, std::array<REAL, 3>{x, y, z}, format, decimal_places);
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
        std::array<REAL, 3> coordinates{};
        detail::read_text_coordinates(input, coordinates, format);
        if (input) {
            *this = {coordinates[0], coordinates[1], coordinates[2]};
        }
        return input;
    }

    /**
     * @brief Write one version-1 tagged `point3` text record.
     *
     * The row contains the decimal tag, `point3` and `v1` tokens, then X/Y/Z
     * coordinates in classic-locale, round-trip scientific notation. It ends
     * with one newline. No explicit flush is requested; normal stream policy
     * applies. Caller formatting and locale are unchanged. The complete
     * grammar is documented in `DATA.md`.
     *
     * @param tag Application-defined record tag; repeated tags are allowed.
     * @param output Destination text stream.
     * @return Reference to output after attempting to write one
     * newline-terminated row.
     * @throws std::invalid_argument When any coordinate is non-finite.
     * @throws std::bad_alloc When buffering the encoded row fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& tag_write(std::size_t tag, std::ostream& output) const {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            throw std::invalid_argument(
                "tagged point3 coordinates must be finite");
        }
        return detail::write_tagged_text_record<REAL>(
            output, tag, "point3", [this](std::ostream& row) {
                row << ' ' << x << ' ' << y << ' ' << z;
            });
    }

    /**
     * @brief Read and allocate one version-1 tagged `point3` text record.
     *
     * When a row is present, exactly that physical row is consumed. Its type
     * token must be `point3`; malformed data and type mismatches consume the
     * row and set `failbit`.
     *
     * @param input Source text stream positioned at the start of a row.
     * @return Tag and non-null shared point after success, or `std::nullopt` at
     * EOF, another read failure, or after a malformed row.
     * @throws std::bad_alloc When buffering the row or allocating the point fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    [[nodiscard]] static std::optional<tagged_read_result<point3>> tag_read(
        std::istream& input);

    /**
     * @brief Write three native `REAL` values in X/Y/Z order as binary data.
     * @param output Destination binary stream.
     * @return Reference to output.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& write(std::ostream& output) const {
        return detail::write_binary_coordinates(
            output, std::array<REAL, 3>{x, y, z});
    }

    /**
     * @brief Read three native `REAL` values in X/Y/Z order from binary data.
     * @param input Source binary stream.
     * @return Reference to input. This point changes only after a complete record.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::istream& read(std::istream& input) {
        std::array<REAL, 3> coordinates{};
        detail::read_binary_coordinates(input, coordinates);
        if (input) {
            *this = {coordinates[0], coordinates[1], coordinates[2]};
        }
        return input;
    }

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
        return index == 0 ? x : (index == 1 ? y : z);
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
        return index == 0 ? x : (index == 1 ? y : z);
    }

    /**
     * @brief Compute distance from the Cartesian world origin.
     * @return Euclidean magnitude in the coordinate units.
     */
    [[nodiscard]] REAL magnitude() const noexcept {
        return std::hypot(x, y, z);
    }

    /**
     * @brief Compute Manhattan distance from the Cartesian world origin.
     * @return Sum of the absolute coordinate values in the coordinate units.
     */
    [[nodiscard]] REAL manhattan_distance() const noexcept {
        return std::abs(x) + std::abs(y) + std::abs(z);
    }

    /**
     * @brief Translate this point in place.
     * @param offset World-space translation vector.
     * @return Reference to this point.
     */
    constexpr point3& operator+=(const vector3<REAL>& offset) noexcept {
        x += offset.x;
        y += offset.y;
        z += offset.z;
        return *this;
    }

    /**
     * @brief Translate this point by the negative of a vector.
     * @param offset World-space translation vector to subtract.
     * @return Reference to this point.
     */
    constexpr point3& operator-=(const vector3<REAL>& offset) noexcept {
        x -= offset.x;
        y -= offset.y;
        z -= offset.z;
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
};

namespace detail {

/** @cond */

template <std::floating_point REAL>
[[nodiscard]] inline std::optional<tagged_read_result<point3<REAL>>>
decode_tagged_point3(
    const tagged_text_record& record,
    std::istream& source) {
    if (record.entity_type != "point3") {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }

    auto payload = tagged_payload_input(record.payload);
    REAL parsed_x = REAL(0);
    REAL parsed_y = REAL(0);
    REAL parsed_z = REAL(0);
    if (!read_real_token(payload, parsed_x) ||
        !read_real_token(payload, parsed_y) ||
        !read_real_token(payload, parsed_z) ||
        !tagged_payload_exhausted(payload)) {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }

    return tagged_read_result<point3<REAL>>{
        record.tag,
        std::make_shared<point3<REAL>>(
            point3<REAL>{parsed_x, parsed_y, parsed_z})};
}

/** @endcond */

} // namespace detail

template <std::floating_point REAL>
std::optional<tagged_read_result<point3<REAL>>> point3<REAL>::tag_read(
    std::istream& input) {
    const auto record = detail::read_tagged_text_record(input);
    if (!record) {
        return std::nullopt;
    }
    return detail::decode_tagged_point3<REAL>(*record, input);
}

/**
 * @brief Write a 3D point as whitespace-separated X, Y, and Z coordinates.
 * @tparam REAL Floating-point scalar type.
 * @param output Destination stream.
 * @param value Point to write.
 * @return Reference to output.
 * @throws std::ios_base::failure When enabled by the stream exception mask.
 */
template <std::floating_point REAL>
std::ostream& operator<<(std::ostream& output, const point3<REAL>& value) {
    return output << value.x << ' ' << value.y << ' ' << value.z;
}

/**
 * @brief Read whitespace-separated X, Y, and Z coordinates into a 3D point.
 * @tparam REAL Floating-point scalar type.
 * @param input Source stream.
 * @param value Point updated only when all coordinates are read successfully.
 * @return Reference to input.
 * @throws std::ios_base::failure When enabled by the stream exception mask.
 */
template <std::floating_point REAL>
std::istream& operator>>(std::istream& input, point3<REAL>& value) {
    return value.csv_read(input, text_format::txt);
}

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
    return {left.x - right.x, left.y - right.y, left.z - right.z};
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
