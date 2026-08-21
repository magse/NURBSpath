#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point3.hpp"
#include "nurbspath/serialization.hpp"

#include <cmath>
#include <concepts>
#include <cstddef>
#include <istream>
#include <memory>
#include <numbers>
#include <optional>
#include <ostream>
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
     * @brief Write one version-1 tagged `sphere3` text record.
     *
     * The row contains the decimal tag, `sphere3` and `v1` tokens, then the
     * center and radius in classic-locale, round-trip scientific notation. It
     * ends with one newline. No explicit flush is requested; normal stream
     * policy applies. Caller formatting and locale are unchanged. The complete
     * grammar is documented in `DATA.md`.
     *
     * @param tag Application-defined record tag; repeated tags are allowed.
     * @param output Destination text stream.
     * @return Reference to output after attempting to write one
     * newline-terminated row.
     * @throws std::invalid_argument When the center or radius is non-finite.
     * @throws std::bad_alloc When buffering the encoded row fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& tag_write(std::size_t tag, std::ostream& output) const {
        if (!std::isfinite(center_.x) || !std::isfinite(center_.y) ||
            !std::isfinite(center_.z) || !std::isfinite(radius_)) {
            throw std::invalid_argument(
                "tagged sphere3 center and radius must be finite");
        }
        return detail::write_tagged_text_record<REAL>(
            output, tag, "sphere3", [this](std::ostream& row) {
                row << ' ' << center_.x << ' ' << center_.y << ' '
                    << center_.z << ' ' << radius_;
            });
    }

    /**
     * @brief Read and allocate one version-1 tagged `sphere3` text record.
     *
     * When a row is present, exactly that physical row is consumed. Its type
     * token must be `sphere3`; malformed data and type mismatches consume the
     * row and set `failbit`.
     *
     * @param input Source text stream positioned at the start of a row.
     * @return Tag and non-null shared sphere after success, or `std::nullopt`
     * at EOF, another read failure, or after a malformed row.
     * @throws std::bad_alloc When buffering the row or allocating the sphere fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    [[nodiscard]] static std::optional<tagged_read_result<sphere3>> tag_read(
        std::istream& input);

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

        REAL u = std::atan2(radial.y, radial.x);
        if (u < REAL(0)) {
            u += REAL(2) * std::numbers::pi_v<REAL>;
        }
        const REAL v = std::asin(std::clamp(radial.z / magnitude, REAL(-1), REAL(1)));
        return {u, v};
    }

private:
    point3<REAL> center_;
    REAL radius_;
};

namespace detail {

/** @cond */

template <std::floating_point REAL>
[[nodiscard]] inline std::optional<tagged_read_result<sphere3<REAL>>>
decode_tagged_sphere3(
    const tagged_text_record& record,
    std::istream& source) {
    if (record.entity_type != "sphere3") {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }

    auto payload = tagged_payload_input(record.payload);
    REAL x = REAL(0);
    REAL y = REAL(0);
    REAL z = REAL(0);
    REAL radius = REAL(0);
    if (!read_real_token(payload, x) || !read_real_token(payload, y) ||
        !read_real_token(payload, z) || !read_real_token(payload, radius) ||
        !(radius > REAL(0)) || !tagged_payload_exhausted(payload)) {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }

    return tagged_read_result<sphere3<REAL>>{
        record.tag,
        std::make_shared<sphere3<REAL>>(point3<REAL>{x, y, z}, radius)};
}

/** @endcond */

} // namespace detail

template <std::floating_point REAL>
std::optional<tagged_read_result<sphere3<REAL>>> sphere3<REAL>::tag_read(
    std::istream& input) {
    const auto record = detail::read_tagged_text_record(input);
    if (!record) {
        return std::nullopt;
    }
    return detail::decode_tagged_sphere3<REAL>(*record, input);
}

} // namespace nurbspath
