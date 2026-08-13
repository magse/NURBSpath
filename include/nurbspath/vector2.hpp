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
 * @brief Cartesian vector in the independent two-dimensional world.
 *
 * A vector is an offset or direction, not a position. It has no implicit
 * relationship to `vector3`; use `project(plane, vector)` to embed it in a
 * selected three-dimensional plane frame.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class vector2 {
public:
    using value_type = REAL; ///< Floating-point scalar type.

    /** @brief Construct the zero vector. */
    constexpr vector2() noexcept = default;

    /**
     * @brief Construct a vector from Cartesian components.
     * @param x_value X component in the 2D world.
     * @param y_value Y component in the 2D world.
     */
    constexpr vector2(REAL x_value, REAL y_value) noexcept
        : x_(x_value), y_(y_value) {}

    /**
     * @brief Compare components exactly.
     * @param other Vector to compare.
     * @return True when both components are exactly equal.
     */
    [[nodiscard]] constexpr bool operator==(const vector2& other) const noexcept = default;

    /** @brief Get the X component. @return X component. */
    [[nodiscard]] constexpr REAL x() const noexcept { return x_; }
    /** @brief Get the Y component. @return Y component. */
    [[nodiscard]] constexpr REAL y() const noexcept { return y_; }

    /** @brief Set the X component. @param value New X component. */
    constexpr void set_x(REAL value) noexcept { x_ = value; }
    /** @brief Set the Y component. @param value New Y component. */
    constexpr void set_y(REAL value) noexcept { y_ = value; }

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
            output, std::array<REAL, 2>{x_, y_}, format, decimal_places);
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
            output, std::array<REAL, 2>{x_, y_});
    }

    /**
     * @brief Read two native `REAL` values in X/Y order from binary data.
     * @param input Source binary stream.
     * @return Reference to input. This vector changes only after a complete record.
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
     * @brief Access a component by index.
     * @param index Component index: 0 for X or 1 for Y.
     * @return Mutable reference to the selected component.
     * @throws std::out_of_range When index is greater than 1.
     */
    [[nodiscard]] constexpr REAL& operator[](std::size_t index) {
        if (index > 1) {
            throw std::out_of_range("vector2 index must be 0 or 1");
        }
        return index == 0 ? x_ : y_;
    }

    /**
     * @brief Access a component by index.
     * @param index Component index: 0 for X or 1 for Y.
     * @return Constant reference to the selected component.
     * @throws std::out_of_range When index is greater than 1.
     */
    [[nodiscard]] constexpr const REAL& operator[](std::size_t index) const {
        if (index > 1) {
            throw std::out_of_range("vector2 index must be 0 or 1");
        }
        return index == 0 ? x_ : y_;
    }

    /** @brief Return this vector unchanged. @return A copy of this vector. */
    [[nodiscard]] constexpr vector2 operator+() const noexcept { return *this; }
    /** @brief Negate every component. @return Negated vector. */
    [[nodiscard]] constexpr vector2 operator-() const noexcept {
        return {-x_, -y_};
    }

    /**
     * @brief Add another vector in place.
     * @param other Vector to add.
     * @return Reference to this vector.
     */
    constexpr vector2& operator+=(const vector2& other) noexcept {
        x_ += other.x_;
        y_ += other.y_;
        return *this;
    }

    /**
     * @brief Subtract another vector in place.
     * @param other Vector to subtract.
     * @return Reference to this vector.
     */
    constexpr vector2& operator-=(const vector2& other) noexcept {
        x_ -= other.x_;
        y_ -= other.y_;
        return *this;
    }

    /**
     * @brief Multiply every component in place.
     * @param scalar Scale factor.
     * @return Reference to this vector.
     */
    constexpr vector2& operator*=(REAL scalar) noexcept {
        x_ *= scalar;
        y_ *= scalar;
        return *this;
    }

    /**
     * @brief Divide every component in place.
     * @param scalar Nonzero divisor.
     * @return Reference to this vector.
     * @throws std::domain_error When scalar is zero.
     */
    constexpr vector2& operator/=(REAL scalar) {
        if (scalar == REAL(0)) {
            throw std::domain_error("cannot divide vector2 by zero");
        }
        x_ /= scalar;
        y_ /= scalar;
        return *this;
    }

    /**
     * @brief Compute the scalar dot product.
     * @param other Second vector.
     * @return Dot product.
     */
    [[nodiscard]] constexpr REAL dot(const vector2& other) const noexcept {
        return x_ * other.x_ + y_ * other.y_;
    }

    /**
     * @brief Compute the signed scalar 2D cross product.
     * @param other Second vector.
     * @return Z component of the corresponding embedded 3D cross product.
     */
    [[nodiscard]] constexpr REAL cross(const vector2& other) const noexcept {
        return x_ * other.y_ - y_ * other.x_;
    }

    /** @brief Compute squared Euclidean length. @return Squared length. */
    [[nodiscard]] constexpr REAL length_squared() const noexcept { return dot(*this); }

    /** @brief Compute Euclidean length. @return Vector length. */
    [[nodiscard]] REAL length() const noexcept { return std::sqrt(length_squared()); }

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
    [[nodiscard]] vector2 normalized(REAL tolerance = default_tolerance()) const {
        const REAL magnitude = length();
        if (magnitude <= tolerance) {
            throw std::domain_error("cannot normalize a near-zero vector2");
        }
        return *this / magnitude;
    }

    /**
     * @brief Rotate this vector counterclockwise by ninety degrees.
     * @return Left perpendicular vector `(-y,x)`.
     */
    [[nodiscard]] constexpr vector2 perpendicular_left() const noexcept {
        return {-y_, x_};
    }

    /**
     * @brief Rotate this vector clockwise by ninety degrees.
     * @return Right perpendicular vector `(y,-x)`.
     */
    [[nodiscard]] constexpr vector2 perpendicular_right() const noexcept {
        return {y_, -x_};
    }

    /**
     * @brief Project this vector onto an axis.
     * @param axis Projection axis; it need not be normalized.
     * @param tolerance Minimum accepted axis length.
     * @return Component parallel to axis.
     * @throws std::domain_error When axis is too small.
     */
    [[nodiscard]] vector2 projected_onto(
        const vector2& axis,
        REAL tolerance = default_tolerance()) const {
        const REAL denominator = axis.length_squared();
        if (denominator <= tolerance * tolerance) {
            throw std::domain_error("cannot project onto a near-zero vector2");
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
    [[nodiscard]] vector2 rejected_from(
        const vector2& axis,
        REAL tolerance = default_tolerance()) const {
        return *this - projected_onto(axis, tolerance);
    }

    /**
     * @brief Reflect this vector across a line with a supplied normal.
     * @param normal Line normal; it need not be normalized.
     * @param tolerance Minimum accepted normal length.
     * @return Reflected vector.
     * @throws std::domain_error When normal is too small.
     */
    [[nodiscard]] vector2 reflected(
        const vector2& normal,
        REAL tolerance = default_tolerance()) const {
        const vector2 unit_normal = normal.normalized(tolerance);
        return *this - REAL(2) * dot(unit_normal) * unit_normal;
    }

    /**
     * @brief Compute the unsigned angle to another vector.
     * @param other Second vector.
     * @param tolerance Minimum accepted product of vector lengths.
     * @return Angle in radians in `[0,pi]`.
     * @throws std::domain_error When either direction is too small.
     */
    [[nodiscard]] REAL angle_to(
        const vector2& other,
        REAL tolerance = default_tolerance()) const {
        const REAL denominator = length() * other.length();
        if (denominator <= tolerance * tolerance) {
            throw std::domain_error("angle is undefined for a near-zero vector2");
        }
        return std::acos(std::clamp(dot(other) / denominator, REAL(-1), REAL(1)));
    }

    /**
     * @brief Compute the signed counterclockwise angle to another vector.
     * @param other Second vector.
     * @param tolerance Minimum accepted vector length.
     * @return Angle in radians in `[-pi,pi]`.
     * @throws std::domain_error When either direction is too small.
     */
    [[nodiscard]] REAL signed_angle_to(
        const vector2& other,
        REAL tolerance = default_tolerance()) const {
        if (is_near_zero(tolerance) || other.is_near_zero(tolerance)) {
            throw std::domain_error("angle is undefined for a near-zero vector2");
        }
        return std::atan2(cross(other), dot(other));
    }

    /**
     * @brief Multiply corresponding components.
     * @param other Second vector.
     * @return Component-wise product.
     */
    [[nodiscard]] constexpr vector2 component_product(
        const vector2& other) const noexcept {
        return {x_ * other.x_, y_ * other.y_};
    }

    /** @brief Get the largest component. @return Largest component value. */
    [[nodiscard]] constexpr REAL max_component() const noexcept {
        return std::max(x_, y_);
    }

    /** @brief Get the smallest component. @return Smallest component value. */
    [[nodiscard]] constexpr REAL min_component() const noexcept {
        return std::min(x_, y_);
    }

    /**
     * @brief Compare vectors using Euclidean distance.
     * @param other Vector to compare.
     * @param tolerance Maximum accepted distance.
     * @return True when vectors differ by at most tolerance.
     */
    [[nodiscard]] bool approximately_equal(
        const vector2& other,
        REAL tolerance = default_tolerance()) const noexcept {
        return (*this - other).length_squared() <= tolerance * tolerance;
    }

    /** @brief Construct the zero vector. @return `(0,0)`. */
    [[nodiscard]] static constexpr vector2 zero() noexcept { return {}; }
    /** @brief Construct the positive X unit vector. @return `(1,0)`. */
    [[nodiscard]] static constexpr vector2 unit_x() noexcept {
        return {REAL(1), REAL(0)};
    }
    /** @brief Construct the positive Y unit vector. @return `(0,1)`. */
    [[nodiscard]] static constexpr vector2 unit_y() noexcept {
        return {REAL(0), REAL(1)};
    }

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
};

/**
 * @brief Write a 2D vector as whitespace-separated X and Y components.
 * @tparam REAL Floating-point scalar type.
 * @param output Destination stream.
 * @param value Vector to write.
 * @return Reference to output.
 * @throws std::ios_base::failure When enabled by the stream exception mask.
 */
template <std::floating_point REAL>
std::ostream& operator<<(std::ostream& output, const vector2<REAL>& value) {
    return output << value.x() << ' ' << value.y();
}

/**
 * @brief Read whitespace-separated X and Y components into a 2D vector.
 * @tparam REAL Floating-point scalar type.
 * @param input Source stream.
 * @param value Vector updated only when both components are read successfully.
 * @return Reference to input.
 * @throws std::ios_base::failure When enabled by the stream exception mask.
 */
template <std::floating_point REAL>
std::istream& operator>>(std::istream& input, vector2<REAL>& value) {
    return value.csv_read(input, text_format::txt);
}

/** @brief Add two vectors. @param left First vector. @param right Second vector. @return Vector sum. */
template <std::floating_point REAL>
[[nodiscard]] constexpr vector2<REAL> operator+(
    vector2<REAL> left,
    const vector2<REAL>& right) noexcept {
    left += right;
    return left;
}

/** @brief Subtract two vectors. @param left First vector. @param right Second vector. @return Vector difference. */
template <std::floating_point REAL>
[[nodiscard]] constexpr vector2<REAL> operator-(
    vector2<REAL> left,
    const vector2<REAL>& right) noexcept {
    left -= right;
    return left;
}

/** @brief Multiply a vector by a scalar. @param value Vector value. @param scalar Scale factor. @return Scaled vector. */
template <std::floating_point REAL>
[[nodiscard]] constexpr vector2<REAL> operator*(
    vector2<REAL> value,
    REAL scalar) noexcept {
    value *= scalar;
    return value;
}

/** @brief Multiply a scalar by a vector. @param scalar Scale factor. @param value Vector value. @return Scaled vector. */
template <std::floating_point REAL>
[[nodiscard]] constexpr vector2<REAL> operator*(
    REAL scalar,
    vector2<REAL> value) noexcept {
    value *= scalar;
    return value;
}

/** @brief Divide a vector by a scalar. @param value Vector value. @param scalar Nonzero divisor. @return Divided vector. @throws std::domain_error When scalar is zero. */
template <std::floating_point REAL>
[[nodiscard]] constexpr vector2<REAL> operator/(vector2<REAL> value, REAL scalar) {
    value /= scalar;
    return value;
}

/** @brief Compute the dot product. @param left First vector. @param right Second vector. @return Dot product. */
template <std::floating_point REAL>
[[nodiscard]] constexpr REAL dot(
    const vector2<REAL>& left,
    const vector2<REAL>& right) noexcept {
    return left.dot(right);
}

/** @brief Compute the signed scalar cross product. @param left First vector. @param right Second vector. @return Signed scalar cross product. */
template <std::floating_point REAL>
[[nodiscard]] constexpr REAL cross(
    const vector2<REAL>& left,
    const vector2<REAL>& right) noexcept {
    return left.cross(right);
}

/** @brief Linearly interpolate vectors. @param first Value at fraction zero. @param second Value at fraction one. @param fraction Interpolation fraction. @return Interpolated vector. */
template <std::floating_point REAL>
[[nodiscard]] constexpr vector2<REAL> lerp(
    const vector2<REAL>& first,
    const vector2<REAL>& second,
    REAL fraction) noexcept {
    return (REAL(1) - fraction) * first + fraction * second;
}

} // namespace nurbspath
