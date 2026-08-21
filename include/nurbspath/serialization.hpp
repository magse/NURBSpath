#pragma once

#include "nurbspath/config.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <ios>
#include <istream>
#include <limits>
#include <locale>
#include <memory>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace nurbspath {

/**
 * @brief Delimited text format used for Cartesian coordinate input and output.
 */
enum class text_format {
    csv, ///< Comma-separated coordinates.
    tsv, ///< Tab-separated coordinates.
    txt  ///< Whitespace-separated coordinates.
};

/**
 * @brief Result of reading one tagged row for a known concrete entity type.
 * @tparam ENTITY Concrete entity type allocated by the reader.
 */
template <typename ENTITY>
struct tagged_read_result {
    std::size_t tag = 0; ///< Application-defined tag stored at the row start.
    /// Shared owner; successful reads always provide a non-null pointer.
    std::shared_ptr<ENTITY> entity;
};

namespace detail {

/** @cond */

class output_format_guard {
public:
    explicit output_format_guard(std::ios_base& output) noexcept
        : output_(output), flags_(output.flags()), precision_(output.precision()) {}

    output_format_guard(const output_format_guard&) = delete;
    output_format_guard& operator=(const output_format_guard&) = delete;

    ~output_format_guard() {
        output_.flags(flags_);
        output_.precision(precision_);
    }

private:
    std::ios_base& output_;
    std::ios_base::fmtflags flags_;
    std::streamsize precision_;
};

struct tagged_text_record {
    std::size_t tag;
    std::string entity_type;
    std::string payload;
};

[[nodiscard]] inline bool parse_size_token(
    std::string_view token,
    std::size_t& value) noexcept {
    std::size_t parsed = 0;
    const auto result = std::from_chars(
        token.data(), token.data() + token.size(), parsed, 10);
    if (result.ec != std::errc{} || result.ptr != token.data() + token.size()) {
        return false;
    }
    value = parsed;
    return true;
}

[[nodiscard]] inline bool valid_tagged_real_token(
    std::string_view token) noexcept {
    std::size_t index = 0;
    if (index < token.size() &&
        (token[index] == '+' || token[index] == '-')) {
        ++index;
    }

    bool has_digit = false;
    while (index < token.size() &&
           token[index] >= '0' && token[index] <= '9') {
        has_digit = true;
        ++index;
    }
    if (index < token.size() && token[index] == '.') {
        ++index;
        while (index < token.size() &&
               token[index] >= '0' && token[index] <= '9') {
            has_digit = true;
            ++index;
        }
    }
    if (!has_digit) {
        return false;
    }

    if (index < token.size() &&
        (token[index] == 'e' || token[index] == 'E')) {
        ++index;
        if (index < token.size() &&
            (token[index] == '+' || token[index] == '-')) {
            ++index;
        }
        const std::size_t exponent_start = index;
        while (index < token.size() &&
               token[index] >= '0' && token[index] <= '9') {
            ++index;
        }
        if (index == exponent_start) {
            return false;
        }
    }
    return index == token.size();
}

template <std::floating_point REAL>
[[nodiscard]] inline bool parse_real_token(
    std::string_view token,
    REAL& value) {
    if (!valid_tagged_real_token(token)) {
        return false;
    }

    REAL parsed = REAL(0);
    if constexpr (std::same_as<REAL, float> || std::same_as<REAL, double>) {
        std::string_view parsed_token = token;
        if (parsed_token.front() == '+') {
            parsed_token.remove_prefix(1);
        }
        const auto result = std::from_chars(
            parsed_token.data(), parsed_token.data() + parsed_token.size(),
            parsed, std::chars_format::general);
        if (result.ec != std::errc{} ||
            result.ptr != parsed_token.data() + parsed_token.size() ||
            !std::isfinite(parsed)) {
            return false;
        }
    } else {
        std::istringstream parser{std::string(token)};
        parser.imbue(std::locale::classic());
        parser >> parsed;
        if (parser.fail()) {
            const REAL magnitude = std::abs(parsed);
            if (!(magnitude > REAL(0) &&
                  magnitude < std::numeric_limits<REAL>::min())) {
                return false;
            }
        }
        if (!std::isfinite(parsed)) {
            return false;
        }
    }

    value = parsed;
    return true;
}

template <std::floating_point REAL>
[[nodiscard]] inline bool read_real_token(
    std::istream& input,
    REAL& value) {
    std::string token;
    return (input >> token) && parse_real_token(token, value);
}

inline void mark_tagged_read_failure(std::istream& input) {
    input.setstate(std::ios::failbit);
}

[[nodiscard]] inline std::optional<tagged_text_record> read_tagged_text_record(
    std::istream& input) {
    std::string row;
    if (!std::getline(input, row)) {
        return std::nullopt;
    }

    std::istringstream row_input(row);
    row_input.imbue(std::locale::classic());
    std::string tag_token;
    std::string entity_type;
    std::string version_token;
    if (!(row_input >> tag_token >> entity_type >> version_token) ||
        version_token != "v1") {
        mark_tagged_read_failure(input);
        return std::nullopt;
    }

    std::size_t tag = 0;
    if (!parse_size_token(tag_token, tag)) {
        mark_tagged_read_failure(input);
        return std::nullopt;
    }

    std::string payload;
    std::getline(row_input, payload);
    return tagged_text_record{
        tag, std::move(entity_type), std::move(payload)};
}

[[nodiscard]] inline std::istringstream tagged_payload_input(
    const std::string& payload) {
    std::istringstream input(payload);
    input.imbue(std::locale::classic());
    return input;
}

[[nodiscard]] inline bool tagged_payload_exhausted(std::istream& input) {
    std::string extra_token;
    return !(input >> extra_token);
}

template <std::floating_point REAL, typename payload_writer>
std::ostream& write_tagged_text_record(
    std::ostream& output,
    std::size_t tag,
    std::string_view entity_type,
    payload_writer&& write_payload) {
    std::ostringstream row;
    row.imbue(std::locale::classic());
    row.setf(std::ios::dec, std::ios::basefield);
    row.setf(std::ios::scientific, std::ios::floatfield);
    row.unsetf(
        std::ios::showbase | std::ios::showpos | std::ios::uppercase |
        std::ios::boolalpha);
    row.precision(std::numeric_limits<REAL>::max_digits10 - 1);
    row << tag << ' ' << entity_type << " v1";
    std::forward<payload_writer>(write_payload)(row);
    row.put('\n');

    const std::string encoded = row.str();
    constexpr std::size_t chunk_limit = 1024U * 1024U;
    std::size_t offset = 0;
    while (offset < encoded.size() && output) {
        const std::size_t preferred_chunk =
            std::min(chunk_limit, encoded.size() - offset);
        const std::streamsize maximum_chunk =
            std::numeric_limits<std::streamsize>::max();
        const std::streamsize chunk_size =
            std::cmp_less_equal(preferred_chunk, maximum_chunk)
                ? static_cast<std::streamsize>(preferred_chunk)
                : maximum_chunk;
        output.write(
            encoded.data() + offset,
            chunk_size);
        offset += static_cast<std::size_t>(chunk_size);
    }
    return output;
}

[[nodiscard]] inline char coordinate_separator(text_format format) {
    switch (format) {
    case text_format::csv:
        return ',';
    case text_format::tsv:
        return '\t';
    case text_format::txt:
        return ' ';
    }
    throw std::invalid_argument("unsupported coordinate text format");
}

template <std::floating_point REAL, std::size_t SIZE>
std::ostream& write_text_coordinates(
    std::ostream& output,
    const std::array<REAL, SIZE>& coordinates,
    text_format format,
    std::streamsize decimal_places) {
    if (decimal_places < 0) {
        throw std::invalid_argument("decimal_places must be nonnegative");
    }
    const char separator = coordinate_separator(format);
    const output_format_guard restore_format(output);
    output.setf(std::ios::scientific, std::ios::floatfield);
    output.precision(decimal_places);
    for (std::size_t index = 0; index < SIZE; ++index) {
        if (index != 0) {
            output.put(separator);
        }
        output << coordinates[index];
    }
    return output;
}

template <std::floating_point REAL, std::size_t SIZE>
std::istream& read_text_coordinates(
    std::istream& input,
    std::array<REAL, SIZE>& coordinates,
    text_format format) {
    const char separator = coordinate_separator(format);
    if (!(input >> coordinates[0])) {
        return input;
    }

    for (std::size_t index = 1; index < SIZE; ++index) {
        if (format != text_format::txt) {
            char actual_separator = '\0';
            if (!input.get(actual_separator)) {
                return input;
            }
            if (actual_separator != separator) {
                input.setstate(std::ios::failbit);
                return input;
            }
        }
        if (!(input >> coordinates[index])) {
            return input;
        }
    }
    return input;
}

template <std::floating_point REAL, std::size_t SIZE>
std::ostream& write_binary_coordinates(
    std::ostream& output,
    const std::array<REAL, SIZE>& coordinates) {
    for (const REAL coordinate : coordinates) {
        output.write(
            reinterpret_cast<const char*>(&coordinate),
            static_cast<std::streamsize>(sizeof(REAL)));
    }
    return output;
}

template <std::floating_point REAL, std::size_t SIZE>
std::istream& read_binary_coordinates(
    std::istream& input,
    std::array<REAL, SIZE>& coordinates) {
    for (REAL& coordinate : coordinates) {
        input.read(
            reinterpret_cast<char*>(&coordinate),
            static_cast<std::streamsize>(sizeof(REAL)));
        if (!input) {
            return input;
        }
    }
    return input;
}

/** @endcond */

} // namespace detail
} // namespace nurbspath
