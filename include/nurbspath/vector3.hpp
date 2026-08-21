#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/serialization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <istream>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>

namespace nurbspath {

/**
 * @brief Cartesian three-dimensional vector.
 *
 * The type deliberately models a vector rather than a position. Keeping
 * vectors and points separate prevents nonsensical position operations. Its
 * components are public, zero-initialized aggregate members.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct vector3 {
    using value_type = REAL; ///< Floating-point scalar type.

    REAL x = REAL(0); ///< X component.
    REAL y = REAL(0); ///< Y component.
    REAL z = REAL(0); ///< Z component.

    /**
     * @brief Compare components exactly.
     * @param other Vector to compare.
     * @return True when all components are exactly equal.
     */
    [[nodiscard]] constexpr bool operator==(const vector3& other) const noexcept = default;

    /**
     * @brief Write the components in CSV, TSV, or whitespace-delimited text.
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
     * @brief Read the components from CSV, TSV, or whitespace-delimited text.
     * @param input Source text stream.
     * @param format Delimited text format; CSV is the default.
     * @return Reference to input. This vector changes only after a complete record.
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
     * @brief Write one version-1 tagged `vector3` text record.
     *
     * The row contains the decimal tag, `vector3` and `v1` tokens, then X/Y/Z
     * components in classic-locale, round-trip scientific notation. It ends
     * with one newline. No explicit flush is requested; normal stream policy
     * applies. Caller formatting and locale are unchanged. The complete
     * grammar is documented in `DATA.md`.
     *
     * @param tag Application-defined record tag; repeated tags are allowed.
     * @param output Destination text stream.
     * @return Reference to output after attempting to write one
     * newline-terminated row.
     * @throws std::invalid_argument When any component is non-finite.
     * @throws std::bad_alloc When buffering the encoded row fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& tag_write(std::size_t tag, std::ostream& output) const {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            throw std::invalid_argument(
                "tagged vector3 components must be finite");
        }
        return detail::write_tagged_text_record<REAL>(
            output, tag, "vector3", [this](std::ostream& row) {
                row << ' ' << x << ' ' << y << ' ' << z;
            });
    }

    /**
     * @brief Read and allocate one version-1 tagged `vector3` text record.
     *
     * When a row is present, exactly that physical row is consumed. Its type
     * token must be `vector3`; malformed data and type mismatches consume the
     * row and set `failbit`.
     *
     * @param input Source text stream positioned at the start of a row.
     * @return Tag and non-null shared vector after success, or `std::nullopt`
     * at EOF, another read failure, or after a malformed row.
     * @throws std::bad_alloc When buffering the row or allocating the vector fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    [[nodiscard]] static std::optional<tagged_read_result<vector3>> tag_read(
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
     * @return Reference to input. This vector changes only after a complete record.
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
     * @brief Access a component by index.
     * @param index Component index: 0 for X, 1 for Y, or 2 for Z.
     * @return Mutable reference to the selected component.
     * @throws std::out_of_range When index is greater than 2.
     */
    [[nodiscard]] constexpr REAL& operator[](std::size_t index) {
        if (index > 2) {
            throw std::out_of_range("vector3 index must be 0, 1, or 2");
        }
        return index == 0 ? x : (index == 1 ? y : z);
    }

    /**
     * @brief Access a component by index.
     * @param index Component index: 0 for X, 1 for Y, or 2 for Z.
     * @return Constant reference to the selected component.
     * @throws std::out_of_range When index is greater than 2.
     */
    [[nodiscard]] constexpr const REAL& operator[](std::size_t index) const {
        if (index > 2) {
            throw std::out_of_range("vector3 index must be 0, 1, or 2");
        }
        return index == 0 ? x : (index == 1 ? y : z);
    }

    /** @brief Return this vector unchanged. @return A copy of this vector. */
    [[nodiscard]] constexpr vector3 operator+() const noexcept { return *this; }
    /** @brief Negate every component. @return Negated vector. */
    [[nodiscard]] constexpr vector3 operator-() const noexcept {
        return {-x, -y, -z};
    }

    /**
     * @brief Add another vector in place.
     * @param other Vector to add.
     * @return Reference to this vector.
     */
    constexpr vector3& operator+=(const vector3& other) noexcept {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    /**
     * @brief Subtract another vector in place.
     * @param other Vector to subtract.
     * @return Reference to this vector.
     */
    constexpr vector3& operator-=(const vector3& other) noexcept {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    /**
     * @brief Multiply every component in place.
     * @param scalar Scale factor.
     * @return Reference to this vector.
     */
    constexpr vector3& operator*=(REAL scalar) noexcept {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    /**
     * @brief Divide every component in place.
     * @param scalar Nonzero divisor.
     * @return Reference to this vector.
     * @throws std::domain_error When scalar is zero.
     */
    constexpr vector3& operator/=(REAL scalar) {
        if (scalar == REAL(0)) {
            throw std::domain_error("cannot divide vector3 by zero");
        }
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    /**
     * @brief Compute the scalar dot product.
     * @param other Second vector.
     * @return Dot product.
     */
    [[nodiscard]] constexpr REAL dot(const vector3& other) const noexcept {
        return x * other.x + y * other.y + z * other.z;
    }

    /**
     * @brief Compute the right-handed cross product.
     * @param other Second vector.
     * @return Vector perpendicular to both operands.
     */
    [[nodiscard]] constexpr vector3 cross(const vector3& other) const noexcept {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    /** @brief Compute squared Euclidean length. @return Squared length. */
    [[nodiscard]] constexpr REAL length_squared() const noexcept {
        return dot(*this);
    }

    /** @brief Compute Euclidean length. @return Vector length. */
    [[nodiscard]] REAL length() const noexcept {
        return std::hypot(x, y, z);
    }

    /**
     * @brief Test whether vector length is within a tolerance of zero.
     * @param tolerance Nonnegative length tolerance.
     * @return True when squared length is at most tolerance squared.
     */
    [[nodiscard]] bool is_near_zero(
        REAL tolerance = default_tolerance()) const noexcept {
        return length_squared() <= tolerance * tolerance;
    }

    /**
     * @brief Return a unit vector in the same direction.
     * @param tolerance Minimum accepted vector length.
     * @return Normalized vector.
     * @throws std::domain_error When length is not greater than tolerance.
     */
    [[nodiscard]] vector3 normalized(
        REAL tolerance = default_tolerance()) const {
        const REAL magnitude = length();
        if (magnitude <= tolerance) {
            throw std::domain_error("cannot normalize a near-zero vector3");
        }
        return *this / magnitude;
    }

    /**
     * @brief Normalize this vector in place using the default vector tolerance.
     *
     * This vector is unchanged when normalization fails.
     *
     * @throws std::domain_error When length is not greater than
     * `default_tolerance()`.
     */
    void normalize() {
        *this = normalized();
    }

    /**
     * @brief Project this vector onto an axis.
     * @param axis Projection axis; it need not be normalized.
     * @param tolerance Minimum accepted axis length.
     * @return Component parallel to axis.
     * @throws std::domain_error When axis is too small.
     */
    [[nodiscard]] vector3 projected_onto(
        const vector3& axis,
        REAL tolerance = default_tolerance()) const {
        const REAL denominator = axis.length_squared();
        if (denominator <= tolerance * tolerance) {
            throw std::domain_error("cannot project onto a near-zero vector3");
        }
        return axis * (dot(axis) / denominator);
    }

    /**
     * @brief Remove the component parallel to an axis.
     * @param axis Rejection axis; it need not be normalized.
     * @param tolerance Minimum accepted axis length.
     * @return Component perpendicular to axis.
     * @throws std::domain_error When axis is too small.
     */
    [[nodiscard]] vector3 rejected_from(
        const vector3& axis,
        REAL tolerance = default_tolerance()) const {
        return *this - projected_onto(axis, tolerance);
    }

    /**
     * @brief Reflect this vector across a plane normal.
     * @param normal Plane normal; it need not be normalized.
     * @param tolerance Minimum accepted normal length.
     * @return Reflected vector.
     * @throws std::domain_error When normal is too small.
     */
    [[nodiscard]] vector3 reflected(
        const vector3& normal,
        REAL tolerance = default_tolerance()) const {
        const vector3 unit_normal = normal.normalized(tolerance);
        return *this - REAL(2) * dot(unit_normal) * unit_normal;
    }

    /**
     * @brief Compute the unsigned angle to another vector.
     * @param other Second vector.
     * @param tolerance Minimum accepted product of vector lengths.
     * @return Angle in radians in the range [0, pi].
     * @throws std::domain_error When either direction is too small.
     */
    [[nodiscard]] REAL angle_to(
        const vector3& other,
        REAL tolerance = default_tolerance()) const {
        const REAL denominator = length() * other.length();
        if (denominator <= tolerance * tolerance) {
            throw std::domain_error("angle is undefined for a near-zero vector3");
        }
        const REAL cosine = std::clamp(dot(other) / denominator, REAL(-1), REAL(1));
        return std::acos(cosine);
    }

    /**
     * @brief Multiply corresponding components.
     * @param other Second vector.
     * @return Component-wise product.
     */
    [[nodiscard]] constexpr vector3 component_product(
        const vector3& other) const noexcept {
        return {x * other.x, y * other.y, z * other.z};
    }

    /** @brief Get the largest component. @return Largest component value. */
    [[nodiscard]] constexpr REAL max_component() const noexcept {
        return std::max({x, y, z});
    }

    /** @brief Get the smallest component. @return Smallest component value. */
    [[nodiscard]] constexpr REAL min_component() const noexcept {
        return std::min({x, y, z});
    }

    /**
     * @brief Compare vectors using Euclidean distance.
     * @param other Vector to compare.
     * @param tolerance Maximum accepted distance.
     * @return True when vectors differ by at most tolerance.
     */
    [[nodiscard]] bool approximately_equal(
        const vector3& other,
        REAL tolerance = default_tolerance()) const noexcept {
        return (*this - other).length_squared() <= tolerance * tolerance;
    }

    /** @brief Construct the zero vector. @return (0,0,0). */
    [[nodiscard]] static constexpr vector3 zero() noexcept { return {}; }
    /** @brief Construct the positive X unit vector. @return (1,0,0). */
    [[nodiscard]] static constexpr vector3 unit_x() noexcept { return {REAL(1), REAL(0), REAL(0)}; }
    /** @brief Construct the positive Y unit vector. @return (0,1,0). */
    [[nodiscard]] static constexpr vector3 unit_y() noexcept { return {REAL(0), REAL(1), REAL(0)}; }
    /** @brief Construct the positive Z unit vector. @return (0,0,1). */
    [[nodiscard]] static constexpr vector3 unit_z() noexcept { return {REAL(0), REAL(0), REAL(1)}; }

    /**
     * @brief Get the default absolute vector tolerance.
     * @return 64 times machine epsilon for REAL.
     */
    [[nodiscard]] static constexpr REAL default_tolerance() noexcept {
        return REAL(64) * std::numeric_limits<REAL>::epsilon();
    }
};

namespace detail {

/** @cond */

template <std::floating_point REAL>
[[nodiscard]] inline std::optional<tagged_read_result<vector3<REAL>>>
decode_tagged_vector3(
    const tagged_text_record& record,
    std::istream& source) {
    if (record.entity_type != "vector3") {
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

    return tagged_read_result<vector3<REAL>>{
        record.tag,
        std::make_shared<vector3<REAL>>(
            vector3<REAL>{parsed_x, parsed_y, parsed_z})};
}

/** @endcond */

} // namespace detail

template <std::floating_point REAL>
std::optional<tagged_read_result<vector3<REAL>>> vector3<REAL>::tag_read(
    std::istream& input) {
    const auto record = detail::read_tagged_text_record(input);
    if (!record) {
        return std::nullopt;
    }
    return detail::decode_tagged_vector3<REAL>(*record, input);
}

/**
 * @brief Write a 3D vector as whitespace-separated X, Y, and Z components.
 * @tparam REAL Floating-point scalar type.
 * @param output Destination stream.
 * @param value Vector to write.
 * @return Reference to output.
 * @throws std::ios_base::failure When enabled by the stream exception mask.
 */
template <std::floating_point REAL>
std::ostream& operator<<(std::ostream& output, const vector3<REAL>& value) {
    return output << value.x << ' ' << value.y << ' ' << value.z;
}

/**
 * @brief Read whitespace-separated X, Y, and Z components into a 3D vector.
 * @tparam REAL Floating-point scalar type.
 * @param input Source stream.
 * @param value Vector updated only when all components are read successfully.
 * @return Reference to input.
 * @throws std::ios_base::failure When enabled by the stream exception mask.
 */
template <std::floating_point REAL>
std::istream& operator>>(std::istream& input, vector3<REAL>& value) {
    return value.csv_read(input, text_format::txt);
}

template <std::floating_point REAL>
/**
 * @brief Add two vectors.
 * @param left First vector.
 * @param right Second vector.
 * @return Vector sum.
 */
[[nodiscard]] constexpr vector3<REAL> operator+(
    vector3<REAL> left,
    const vector3<REAL>& right) noexcept {
    left += right;
    return left;
}

template <std::floating_point REAL>
/**
 * @brief Subtract two vectors.
 * @param left First vector.
 * @param right Second vector.
 * @return Vector difference.
 */
[[nodiscard]] constexpr vector3<REAL> operator-(
    vector3<REAL> left,
    const vector3<REAL>& right) noexcept {
    left -= right;
    return left;
}

template <std::floating_point REAL>
/**
 * @brief Multiply a vector by a scalar.
 * @param value Vector value.
 * @param scalar Scale factor.
 * @return Scaled vector.
 */
[[nodiscard]] constexpr vector3<REAL> operator*(
    vector3<REAL> value,
    REAL scalar) noexcept {
    value *= scalar;
    return value;
}

template <std::floating_point REAL>
/**
 * @brief Multiply a scalar by a vector.
 * @param scalar Scale factor.
 * @param value Vector value.
 * @return Scaled vector.
 */
[[nodiscard]] constexpr vector3<REAL> operator*(
    REAL scalar,
    vector3<REAL> value) noexcept {
    value *= scalar;
    return value;
}

template <std::floating_point REAL>
/**
 * @brief Divide a vector by a scalar.
 * @param value Vector value.
 * @param scalar Nonzero divisor.
 * @return Divided vector.
 * @throws std::domain_error When scalar is zero.
 */
[[nodiscard]] constexpr vector3<REAL> operator/(
    vector3<REAL> value,
    REAL scalar) {
    value /= scalar;
    return value;
}

template <std::floating_point REAL>
/**
 * @brief Compute the dot product of two vectors.
 * @param left First vector.
 * @param right Second vector.
 * @return Dot product.
 */
[[nodiscard]] constexpr REAL dot(
    const vector3<REAL>& left,
    const vector3<REAL>& right) noexcept {
    return left.dot(right);
}

template <std::floating_point REAL>
/**
 * @brief Compute the cross product of two vectors.
 * @param left First vector.
 * @param right Second vector.
 * @return Right-handed cross product.
 */
[[nodiscard]] constexpr vector3<REAL> cross(
    const vector3<REAL>& left,
    const vector3<REAL>& right) noexcept {
    return left.cross(right);
}

template <std::floating_point REAL>
/**
 * @brief Linearly interpolate two vectors.
 * @param first Value at fraction zero.
 * @param second Value at fraction one.
 * @param fraction Interpolation fraction; extrapolation is permitted.
 * @return Interpolated vector.
 */
[[nodiscard]] constexpr vector3<REAL> lerp(
    const vector3<REAL>& first,
    const vector3<REAL>& second,
    REAL fraction) noexcept {
    return (REAL(1) - fraction) * first + fraction * second;
}

} // namespace nurbspath
