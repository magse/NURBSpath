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
#include <ostream>
#include <stdexcept>

namespace nurbspath {

/**
 * @brief Cartesian three-dimensional vector.
 *
 * The type deliberately models a vector rather than a position. Keeping
 * vectors and points separate prevents nonsensical position operations.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class vector3 {
public:
    using value_type = REAL; ///< Floating-point scalar type.

    /** @brief Construct the zero vector. */
    constexpr vector3() noexcept = default;

    /**
     * @brief Construct a vector from Cartesian components.
     * @param x_value X component.
     * @param y_value Y component.
     * @param z_value Z component.
     */
    constexpr vector3(REAL x_value, REAL y_value, REAL z_value) noexcept
        : x_(x_value), y_(y_value), z_(z_value) {}

    /**
     * @brief Compare components exactly.
     * @param other Vector to compare.
     * @return True when all components are exactly equal.
     */
    [[nodiscard]] constexpr bool operator==(const vector3& other) const noexcept = default;

    /** @brief Get the X component. @return X component. */
    [[nodiscard]] constexpr REAL x() const noexcept { return x_; }
    /** @brief Get the Y component. @return Y component. */
    [[nodiscard]] constexpr REAL y() const noexcept { return y_; }
    /** @brief Get the Z component. @return Z component. */
    [[nodiscard]] constexpr REAL z() const noexcept { return z_; }

    /** @brief Set the X component. @param value New X component. */
    constexpr void set_x(REAL value) noexcept { x_ = value; }
    /** @brief Set the Y component. @param value New Y component. */
    constexpr void set_y(REAL value) noexcept { y_ = value; }
    /** @brief Set the Z component. @param value New Z component. */
    constexpr void set_z(REAL value) noexcept { z_ = value; }

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
            output, std::array<REAL, 3>{x_, y_, z_}, format, decimal_places);
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
     * @brief Write three native `REAL` values in X/Y/Z order as binary data.
     * @param output Destination binary stream.
     * @return Reference to output.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& write(std::ostream& output) const {
        return detail::write_binary_coordinates(
            output, std::array<REAL, 3>{x_, y_, z_});
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
        return index == 0 ? x_ : (index == 1 ? y_ : z_);
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
        return index == 0 ? x_ : (index == 1 ? y_ : z_);
    }

    /** @brief Return this vector unchanged. @return A copy of this vector. */
    [[nodiscard]] constexpr vector3 operator+() const noexcept { return *this; }
    /** @brief Negate every component. @return Negated vector. */
    [[nodiscard]] constexpr vector3 operator-() const noexcept {
        return {-x_, -y_, -z_};
    }

    /**
     * @brief Add another vector in place.
     * @param other Vector to add.
     * @return Reference to this vector.
     */
    constexpr vector3& operator+=(const vector3& other) noexcept {
        x_ += other.x_;
        y_ += other.y_;
        z_ += other.z_;
        return *this;
    }

    /**
     * @brief Subtract another vector in place.
     * @param other Vector to subtract.
     * @return Reference to this vector.
     */
    constexpr vector3& operator-=(const vector3& other) noexcept {
        x_ -= other.x_;
        y_ -= other.y_;
        z_ -= other.z_;
        return *this;
    }

    /**
     * @brief Multiply every component in place.
     * @param scalar Scale factor.
     * @return Reference to this vector.
     */
    constexpr vector3& operator*=(REAL scalar) noexcept {
        x_ *= scalar;
        y_ *= scalar;
        z_ *= scalar;
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
        x_ /= scalar;
        y_ /= scalar;
        z_ /= scalar;
        return *this;
    }

    /**
     * @brief Compute the scalar dot product.
     * @param other Second vector.
     * @return Dot product.
     */
    [[nodiscard]] constexpr REAL dot(const vector3& other) const noexcept {
        return x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
    }

    /**
     * @brief Compute the right-handed cross product.
     * @param other Second vector.
     * @return Vector perpendicular to both operands.
     */
    [[nodiscard]] constexpr vector3 cross(const vector3& other) const noexcept {
        return {
            y_ * other.z_ - z_ * other.y_,
            z_ * other.x_ - x_ * other.z_,
            x_ * other.y_ - y_ * other.x_
        };
    }

    /** @brief Compute squared Euclidean length. @return Squared length. */
    [[nodiscard]] constexpr REAL length_squared() const noexcept {
        return dot(*this);
    }

    /** @brief Compute Euclidean length. @return Vector length. */
    [[nodiscard]] REAL length() const noexcept {
        return std::sqrt(length_squared());
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
        return {x_ * other.x_, y_ * other.y_, z_ * other.z_};
    }

    /** @brief Get the largest component. @return Largest component value. */
    [[nodiscard]] constexpr REAL max_component() const noexcept {
        return std::max({x_, y_, z_});
    }

    /** @brief Get the smallest component. @return Smallest component value. */
    [[nodiscard]] constexpr REAL min_component() const noexcept {
        return std::min({x_, y_, z_});
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

private:
    REAL x_ = REAL(0);
    REAL y_ = REAL(0);
    REAL z_ = REAL(0);
};

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
    return output << value.x() << ' ' << value.y() << ' ' << value.z();
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
