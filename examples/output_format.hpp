#pragma once

#include <nurbspath/config.hpp>

#include <cmath>
#include <iomanip>
#include <locale>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>

namespace nurbspath_examples {
namespace detail {

[[nodiscard]] inline bool is_space(char value) {
    return value == ' ' || value == '\t' || value == '\r' ||
           value == '\n' || value == '\f' || value == '\v';
}

// Preserve integer tokens, identifiers, and anything that is not entirely a
// finite floating-point number. Formatting here never changes writer precision.
inline void write_numeric_token(std::ostream& output, std::string_view token) {
    if (token.find_first_of(".eE") != std::string_view::npos) {
        std::istringstream input{std::string(token)};
        input.imbue(std::locale::classic());
        long double value = 0.0L;
        if ((input >> std::noskipws >> value) &&
            input.peek() == std::char_traits<char>::eof() &&
            std::isfinite(value)) {
            std::ostringstream formatted;
            formatted.imbue(std::locale::classic());
            formatted << std::scientific << std::setprecision(4) << value;
            output << formatted.str();
            return;
        }
    }
    output << token;
}

inline void write_preview_tokens(std::ostream& output, std::string_view values) {
    std::size_t position = 0;
    while (position < values.size()) {
        if (is_space(values[position]) || values[position] == ',') {
            output.put(values[position++]);
        } else if (values[position] == '"') {
            const std::size_t begin = position++;
            while (position < values.size()) {
                if (values[position] == '\\' && position + 1 < values.size()) {
                    position += 2;
                } else if (values[position++] == '"') {
                    break;
                }
            }
            output << values.substr(begin, position - begin);
        } else {
            const std::size_t begin = position;
            while (position < values.size() && !is_space(values[position]) &&
                   values[position] != ',' && values[position] != '"') {
                ++position;
            }
            write_numeric_token(output, values.substr(begin, position - begin));
        }
    }
}

[[nodiscard]] inline bool is_numeric_svg_attribute(std::string_view name) {
    return name == "x" || name == "y" || name == "x1" || name == "y1" ||
           name == "x2" || name == "y2" || name == "cx" || name == "cy" ||
           name == "r" || name == "stroke-width" ||
           name == "stroke-opacity" || name == "font-size" ||
           name == "stroke-dasharray" || name == "width" ||
           name == "height" || name == "viewBox";
}

} // namespace detail

// Display a rounded copy while leaving canonical rows available for reading.
// Application tokens and quoted text, including all escapes, are copied intact.
inline void write_tagged_preview(std::ostream& output, std::string_view rows) {
    std::size_t position = 0;
    while (position < rows.size()) {
        const std::size_t line_end = rows.find('\n', position);
        const std::size_t end = line_end == std::string_view::npos
                                    ? rows.size() : line_end;
        const auto row = rows.substr(position, end - position);
        const std::size_t first = row.find_first_not_of(" \t\r");
        const std::size_t first_end = row.find_first_of(" \t\r", first);
        if (first != std::string_view::npos &&
            row.substr(first, first_end - first) == "application") {
            output << row;
        } else {
            detail::write_preview_tokens(output, row);
        }
        if (line_end != std::string_view::npos) {
            output.put('\n');
        }
        position = end + (line_end != std::string_view::npos ? 1 : 0);
    }
}

// Only known numeric attributes of generated SVG are rounded. XML declarations,
// namespaces, colors, identifiers, and text content retain their original bytes.
inline void write_svg_preview(std::ostream& output, std::string_view svg) {
    std::size_t position = 0;
    bool inside_tag = false;
    while (position < svg.size()) {
        const char current = svg[position];
        if (current == '<') {
            inside_tag = true;
        } else if (current == '>') {
            inside_tag = false;
        } else if (inside_tag && current == '=') {
            std::size_t name_end = position;
            while (name_end > 0 && detail::is_space(svg[name_end - 1])) {
                --name_end;
            }
            std::size_t name_begin = name_end;
            while (name_begin > 0 &&
                   !detail::is_space(svg[name_begin - 1]) &&
                   svg[name_begin - 1] != '<') {
                --name_begin;
            }
            const auto name = svg.substr(name_begin, name_end - name_begin);
            std::size_t value_begin = position + 1;
            while (value_begin < svg.size() &&
                   detail::is_space(svg[value_begin])) {
                ++value_begin;
            }
            if (value_begin < svg.size() &&
                (svg[value_begin] == '"' || svg[value_begin] == '\'')) {
                const std::size_t value_end = svg.find(
                    svg[value_begin], value_begin + 1);
                if (value_end != std::string_view::npos) {
                    output << svg.substr(position, value_begin - position + 1);
                    const auto value = svg.substr(
                        value_begin + 1, value_end - value_begin - 1);
                    if (detail::is_numeric_svg_attribute(name)) {
                        detail::write_preview_tokens(output, value);
                    } else {
                        output << value;
                    }
                    output.put(svg[value_end]);
                    position = value_end + 1;
                    continue;
                }
            }
        }
        output.put(current);
        ++position;
    }
}

} // namespace nurbspath_examples
