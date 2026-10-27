#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/serialization.hpp"

#include <cmath>
#include <concepts>
#include <cstddef>
#include <istream>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace nurbspath {

namespace detail {

/** @cond */

template <std::floating_point REAL>
[[nodiscard]] bool valid_color_component(REAL value) noexcept {
    return std::isfinite(value) && value >= REAL(0) && value <= REAL(1);
}

template <std::floating_point REAL>
[[nodiscard]] bool valid_line_font(REAL width, REAL on, REAL off) noexcept {
    return std::isfinite(width) && width > REAL(0) &&
           std::isfinite(on) && std::isfinite(off) &&
           ((on == REAL(0) && off == REAL(0)) ||
            (on > REAL(0) && off > REAL(0)));
}

[[nodiscard]] inline bool valid_label_text(std::string_view value) noexcept {
    for (const unsigned char character : value) {
        if ((character < 32 && character != '\n' && character != '\r' &&
             character != '\t') || character == 127) {
            return false;
        }
    }
    return true;
}

inline void write_quoted_label(std::ostream& output, std::string_view value) {
    output.put('"');
    for (const char character : value) {
        switch (character) {
        case '\\': output << "\\\\"; break;
        case '"': output << "\\\""; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default: output.put(character); break;
        }
    }
    output.put('"');
}

[[nodiscard]] inline bool read_quoted_label(
    std::istream& input, std::string& value) {
    input >> std::ws;
    char character = '\0';
    if (!input.get(character) || character != '"') {
        return false;
    }
    while (input.get(character)) {
        if (character == '"') {
            return true;
        }
        if (character == '\\') {
            if (!input.get(character)) {
                return false;
            }
            switch (character) {
            case '\\': value.push_back('\\'); break;
            case '"': value.push_back('"'); break;
            case 'n': value.push_back('\n'); break;
            case 'r': value.push_back('\r'); break;
            case 't': value.push_back('\t'); break;
            default: return false;
            }
        } else {
            const auto byte = static_cast<unsigned char>(character);
            if (byte < 32 || byte == 127) {
                return false;
            }
            value.push_back(character);
        }
    }
    return false;
}

/** @endcond */

} // namespace detail

/**
 * @brief Optional normalized RGBA line color associated with an application tag.
 *
 * Components must be finite and in [0, 1]; alpha is opacity (0 transparent,
 * 1 opaque). This record carries presentation data for a consumer and does not
 * change geometry or SVG drawing state. Omit it to use the consumer's defaults.
 * @tparam REAL Floating-point scalar type used for the color components.
 */
template <std::floating_point REAL>
struct line_color {
    REAL red = REAL(0); ///< Normalized red component in [0, 1].
    REAL green = REAL(0); ///< Normalized green component in [0, 1].
    REAL blue = REAL(0); ///< Normalized blue component in [0, 1].
    REAL alpha = REAL(1); ///< Normalized opacity in [0, 1].

    /**
     * @brief Write one version-1 `linecolor` row without changing stream format.
     * @param tag Application tag associating this color with line elements.
     * @param output Destination stream; one newline-terminated row is appended.
     * @return The destination stream.
     * @throws std::invalid_argument If any component is not finite or in [0, 1].
     * @throws std::bad_alloc If row buffering fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& tag_write(std::size_t tag, std::ostream& output) const {
        if (!detail::valid_color_component(red) ||
            !detail::valid_color_component(green) ||
            !detail::valid_color_component(blue) ||
            !detail::valid_color_component(alpha)) {
            throw std::invalid_argument("line color components must be in [0, 1]");
        }
        return detail::write_tagged_text_record<REAL>(
            output, tag, "linecolor", [this](std::ostream& row) {
                row << ' ' << red << ' ' << green << ' ' << blue << ' ' << alpha;
            });
    }

    /**
     * @brief Read one `linecolor v1` row and allocate a validated color.
     * Advisory application and diagnostic rows are skipped without decoding.
     * @param input Source stream positioned at the start of a physical row.
     * @return Preserved tag and non-null color owner, or nullopt on EOF/failure.
     * Malformed rows set failbit and consume only their own physical row.
     * @throws std::bad_alloc If buffering or allocation fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    [[nodiscard]] static std::optional<tagged_read_result<line_color>> tag_read(
        std::istream& input);
};

/**
 * @brief Optional line width and repeating dash lengths for tagged line elements.
 *
 * Width must be finite and positive. On/off lengths must both be zero for a
 * solid line or both finite and positive for a dashed line. All three lengths
 * use consumer-defined display units; no world or pixel unit is implied.
 * This metadata does not change geometry or SVG drawing state.
 * @tparam REAL Floating-point scalar type used for the display lengths.
 */
template <std::floating_point REAL>
struct line_font {
    REAL width = REAL(1); ///< Positive stroke width in consumer display units.
    REAL on = REAL(0); ///< Visible dash length; zero with off for a solid line.
    REAL off = REAL(0); ///< Gap length; zero with on for a solid line.

    /**
     * @brief Write one version-1 `linefont` row without changing stream format.
     * @param tag Application tag associating this font with line elements.
     * @param output Destination stream; one newline-terminated row is appended.
     * @return The destination stream.
     * @throws std::invalid_argument If width or dash lengths violate the contract.
     * @throws std::bad_alloc If row buffering fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& tag_write(std::size_t tag, std::ostream& output) const {
        if (!detail::valid_line_font(width, on, off)) {
            throw std::invalid_argument("invalid line width or dash lengths");
        }
        return detail::write_tagged_text_record<REAL>(
            output, tag, "linefont", [this](std::ostream& row) {
                row << ' ' << width << ' ' << on << ' ' << off;
            });
    }

    /**
     * @brief Read one `linefont v1` row and allocate a validated line font.
     * Advisory application and diagnostic rows are skipped without decoding.
     * @param input Source stream positioned at the start of a physical row.
     * @return Preserved tag and non-null font owner, or nullopt on EOF/failure.
     * Malformed rows set failbit and consume only their own physical row.
     * @throws std::bad_alloc If buffering or allocation fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    [[nodiscard]] static std::optional<tagged_read_result<line_font>> tag_read(
        std::istream& input);
};

/**
 * @brief Optional sized text associated with an application tag.
 *
 * Size is finite and positive in consumer-defined display units. The consumer
 * chooses placement and font. Quoting escapes backslash, double quote, newline,
 * carriage return, and tab so each record occupies one physical row. Other
 * ASCII control bytes are invalid; bytes above ASCII are preserved verbatim
 * without encoding validation. This metadata introduces no coordinate frame.
 * @tparam REAL Floating-point scalar type used for the display size.
 */
template <std::floating_point REAL>
struct text_label {
    REAL size = REAL(12); ///< Positive text size in consumer display units.
    std::string value; ///< Label text, optionally including escaped line breaks.

    /**
     * @brief Write one version-1 `text` row with a quoted, escaped string.
     * @param tag Application tag associating this label with geometry.
     * @param output Destination stream; one newline-terminated row is appended.
     * @return The destination stream, with its formatting preserved.
     * @throws std::invalid_argument If size or text violates the class contract.
     * @throws std::bad_alloc If row buffering fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& tag_write(std::size_t tag, std::ostream& output) const {
        if (!std::isfinite(size) || !(size > REAL(0)) ||
            !detail::valid_label_text(value)) {
            throw std::invalid_argument("invalid text size or control byte");
        }
        return detail::write_tagged_text_record<REAL>(
            output, tag, "text", [this](std::ostream& row) {
                row << ' ' << size << ' ';
                detail::write_quoted_label(row, value);
            });
    }

    /**
     * @brief Read one `text v1` row and allocate a validated text label.
     * Advisory application and diagnostic rows are skipped without decoding.
     * @param input Source stream positioned at the start of a physical row.
     * @return Preserved tag and non-null label owner, or nullopt on EOF/failure.
     * Invalid quoting, escapes, fields, or values set failbit after that row.
     * @throws std::bad_alloc If buffering or allocation fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    [[nodiscard]] static std::optional<tagged_read_result<text_label>> tag_read(
        std::istream& input);
};

namespace detail {

/** @cond */

template <std::floating_point REAL>
[[nodiscard]] std::optional<tagged_read_result<line_color<REAL>>>
decode_tagged_line_color(const tagged_text_record& record, std::istream& source) {
    auto payload = tagged_payload_input(record.payload);
    line_color<REAL> value;
    if (record.entity_type != "linecolor" ||
        !read_real_token(payload, value.red) ||
        !read_real_token(payload, value.green) ||
        !read_real_token(payload, value.blue) ||
        !read_real_token(payload, value.alpha) ||
        !valid_color_component(value.red) ||
        !valid_color_component(value.green) ||
        !valid_color_component(value.blue) ||
        !valid_color_component(value.alpha) || !tagged_payload_exhausted(payload)) {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }
    return tagged_read_result<line_color<REAL>>{
        record.tag, std::make_shared<line_color<REAL>>(value)};
}

template <std::floating_point REAL>
[[nodiscard]] std::optional<tagged_read_result<line_font<REAL>>>
decode_tagged_line_font(const tagged_text_record& record, std::istream& source) {
    auto payload = tagged_payload_input(record.payload);
    line_font<REAL> value;
    if (record.entity_type != "linefont" ||
        !read_real_token(payload, value.width) ||
        !read_real_token(payload, value.on) ||
        !read_real_token(payload, value.off) ||
        !valid_line_font(value.width, value.on, value.off) ||
        !tagged_payload_exhausted(payload)) {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }
    return tagged_read_result<line_font<REAL>>{
        record.tag, std::make_shared<line_font<REAL>>(value)};
}

template <std::floating_point REAL>
[[nodiscard]] std::optional<tagged_read_result<text_label<REAL>>>
decode_tagged_text_label(const tagged_text_record& record, std::istream& source) {
    auto payload = tagged_payload_input(record.payload);
    text_label<REAL> value;
    if (record.entity_type != "text" || !read_real_token(payload, value.size) ||
        !(value.size > REAL(0)) || !read_quoted_label(payload, value.value) ||
        !tagged_payload_exhausted(payload)) {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }
    return tagged_read_result<text_label<REAL>>{
        record.tag, std::make_shared<text_label<REAL>>(std::move(value))};
}

/** @endcond */

} // namespace detail

template <std::floating_point REAL>
std::optional<tagged_read_result<line_color<REAL>>> line_color<REAL>::tag_read(
    std::istream& input) {
    const auto record = detail::read_tagged_text_record(input);
    return record ? detail::decode_tagged_line_color<REAL>(*record, input)
                  : std::nullopt;
}

template <std::floating_point REAL>
std::optional<tagged_read_result<line_font<REAL>>> line_font<REAL>::tag_read(
    std::istream& input) {
    const auto record = detail::read_tagged_text_record(input);
    return record ? detail::decode_tagged_line_font<REAL>(*record, input)
                  : std::nullopt;
}

template <std::floating_point REAL>
std::optional<tagged_read_result<text_label<REAL>>> text_label<REAL>::tag_read(
    std::istream& input) {
    const auto record = detail::read_tagged_text_record(input);
    return record ? detail::decode_tagged_text_label<REAL>(*record, input)
                  : std::nullopt;
}

} // namespace nurbspath
