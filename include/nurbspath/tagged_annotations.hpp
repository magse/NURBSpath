#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point3.hpp"
#include "nurbspath/tagged_attributes.hpp"

#include <cmath>
#include <concepts>
#include <cstddef>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace nurbspath {

namespace detail {

/** @cond */

[[nodiscard]] inline bool valid_application_token(std::string_view value) noexcept {
    if (value.empty()) {
        return false;
    }
    for (const unsigned char character : value) {
        if (character <= 32 || character == 127) {
            return false;
        }
    }
    return true;
}

/** @endcond */

} // namespace detail

/**
 * @brief Optional output identification of the application producing a stream.
 *
 * Name and version are nonempty, unquoted tokens without ASCII whitespace or
 * control bytes. Bytes above ASCII are preserved without encoding validation.
 * This untagged record may occur anywhere and establishes no reader requirement
 * or geometry state. Built-in tagged readers skip it without decoding its
 * payload. Default empty fields must be populated before writing.
 */
struct application_info {
    std::string name; ///< Nonempty application-name token.
    std::string version; ///< Nonempty application-version token.

    /**
     * @brief Write one optional `application v1 <name> <version>` output row.
     * @param output Destination stream; one newline-terminated row is appended.
     * @return The destination stream, with its formatting preserved.
     * @throws std::invalid_argument If either token is empty or contains ASCII
     * whitespace or control bytes; no row is written in this case.
     * @throws std::bad_alloc If row buffering fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& write(std::ostream& output) const {
        if (!detail::valid_application_token(name) ||
            !detail::valid_application_token(version)) {
            throw std::invalid_argument("invalid application name or version token");
        }
        return detail::write_text_record<double>(
            output, [this](std::ostream& row) {
                row << "application v1 " << name << ' ' << version;
            });
    }
};

/** @brief Severity selecting an optional positioned diagnostic output token. */
enum class diagnostic_level {
    warning, ///< Emit a `warning` annotation.
    information, ///< Emit an `information` annotation.
    error ///< Emit an `error` annotation without changing program or stream state.
};

/**
 * @brief Optional output diagnostic anchored at a position in the 3D world.
 *
 * Coordinates must be finite and use the geometry's world units. Size must be
 * finite and positive in consumer-selected display units. The consumer chooses
 * font, orientation, and display behavior. Quoted text uses the same escapes and
 * byte validation as text_label. The default message is informational, at the
 * world origin, with display size 12 and empty text.
 *
 * These output-only annotations do not alter geometry or establish SVG drawing
 * state. Readers may ignore them; built-in tagged readers skip their complete
 * physical rows without decoding their payloads or returning a value for them.
 * @tparam REAL Floating-point scalar type for coordinates and display size.
 */
template <std::floating_point REAL>
struct diagnostic_message3 {
    diagnostic_level level = diagnostic_level::information; ///< Output severity.
    point3<REAL> position{}; ///< Finite world-space position of the annotation.
    REAL size = REAL(12); ///< Positive text size in consumer display units.
    std::string value; ///< Text, optionally containing escaped line breaks.

    /**
     * @brief Write an optional positioned warning, information, or error row.
     * @param tag Application tag associating this diagnostic with geometry.
     * @param output Destination stream; one newline-terminated row is appended.
     * @return The destination stream, with its formatting preserved.
     * @throws std::invalid_argument If severity, coordinates, size, or text
     * violate the contract; no row is written in this case.
     * @throws std::bad_alloc If row buffering fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& tag_write(std::size_t tag, std::ostream& output) const {
        std::string_view type;
        switch (level) {
        case diagnostic_level::warning: type = "warning"; break;
        case diagnostic_level::information: type = "information"; break;
        case diagnostic_level::error: type = "error"; break;
        default: throw std::invalid_argument("invalid diagnostic level");
        }
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z) || !std::isfinite(size) ||
            !(size > REAL(0)) || !detail::valid_label_text(value)) {
            throw std::invalid_argument("invalid diagnostic position, size, or text");
        }
        return detail::write_tagged_text_record<REAL>(
            output, tag, type, [this](std::ostream& row) {
                row << ' ' << position.x << ' ' << position.y << ' '
                    << position.z << ' ' << size << ' ';
                detail::write_quoted_label(row, value);
            });
    }
};

} // namespace nurbspath
