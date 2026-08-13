#pragma once

#include "nurbspath/config.hpp"

#include <array>
#include <concepts>
#include <cstddef>
#include <ios>
#include <istream>
#include <ostream>
#include <stdexcept>

namespace nurbspath {

/**
 * @brief Delimited text format used for Cartesian coordinate input and output.
 */
enum class text_format {
    csv, ///< Comma-separated coordinates.
    tsv, ///< Tab-separated coordinates.
    txt  ///< Whitespace-separated coordinates.
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
