#include <nurbspath/config.hpp>
#include <nurbspath/tagged_attributes.hpp>
#include <nurbspath/tagged_serialization.hpp>

#include "test_support.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

namespace {

struct comma_decimal : std::numpunct<char> {
    char do_decimal_point() const override { return ','; }
};

void check_invalid(std::string_view row) {
    std::istringstream input{std::string(row)};
    const auto record = nurbspath::tag_read_record<double>(input);
    test_support::check(!record && input.fail(), row);
}

template <typename ATTRIBUTE>
void check_invalid_write(const ATTRIBUTE& attribute, std::string_view message) {
    std::ostringstream output;
    output << "unchanged";
    bool rejected = false;
    try {
        attribute.tag_write(1, output);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    test_support::check(rejected && output.str() == "unchanged", message);
}

template <std::floating_point REAL>
void check_scalar_round_trip() {
    const nurbspath::line_color<REAL> color{
        REAL(0.1), REAL(0.2), REAL(0.3), REAL(0.4)};
    const nurbspath::line_font<REAL> font{REAL(0.3), REAL(1.1), REAL(2.2)};
    const nurbspath::text_label<REAL> label{REAL(1.3), "label"};
    std::stringstream rows;
    color.tag_write(7, rows);
    font.tag_write(7, rows);
    label.tag_write(7, rows);
    const auto decoded_color = nurbspath::line_color<REAL>::tag_read(rows);
    const auto decoded_font = nurbspath::line_font<REAL>::tag_read(rows);
    const auto decoded_label = nurbspath::text_label<REAL>::tag_read(rows);
    test_support::check(decoded_color && decoded_color->tag == 7 &&
        decoded_color->entity->red == color.red &&
        decoded_color->entity->green == color.green &&
        decoded_color->entity->blue == color.blue &&
        decoded_color->entity->alpha == color.alpha,
        "color round trips every component for each scalar type");
    test_support::check(decoded_font && decoded_font->tag == 7 &&
        decoded_font->entity->width == font.width &&
        decoded_font->entity->on == font.on &&
        decoded_font->entity->off == font.off,
        "line font round trips every field for each scalar type");
    test_support::check(decoded_label && decoded_label->tag == 7 &&
        decoded_label->entity->size == label.size &&
        decoded_label->entity->value == label.value,
        "label round trips size and text for each scalar type");
}

} // namespace

int main() {
    using namespace test_support;
    using nurbspath::line_color;
    using nurbspath::line_font;
    using nurbspath::text_label;

    static_assert(std::is_aggregate_v<line_color<real>>);
    static_assert(std::is_aggregate_v<line_font<real>>);
    static_assert(std::is_aggregate_v<text_label<real>>);
    static_assert(std::variant_size_v<nurbspath::entity3_ptr<real>> == 4);
    static_assert(std::variant_size_v<nurbspath::tagged_record_ptr<real>> == 7);

    std::ostringstream output;
    output.imbue(std::locale(std::locale::classic(), new comma_decimal));
    output << std::hex << std::showpos << std::uppercase << std::fixed;
    output.precision(2);
    output.width(8);
    output.fill('_');
    const auto flags = output.flags();
    const auto locale = output.getloc();
    const auto& returned = line_color<real>{1.0, 0.5, 0.0, 0.25}
                               .tag_write(42, output);
    line_font<real>{1.5, 4.0, 2.0}.tag_write(42, output);
    text_label<real>{12.0, "Path \"A\"\\next\n\r\t"}.tag_write(42, output);
    check(&returned == &output, "attribute writer returns destination stream");
    check(output.flags() == flags && output.precision() == 2 &&
              output.width() == 8 && output.fill() == '_' &&
              output.getloc() == locale,
          "attribute writers preserve caller formatting and locale");
    const std::string expected =
        "42 linecolor v1 1.0000000000000000e+00 5.0000000000000000e-01 "
        "0.0000000000000000e+00 2.5000000000000000e-01\n"
        "42 linefont v1 1.5000000000000000e+00 4.0000000000000000e+00 "
        "2.0000000000000000e+00\n"
        "42 text v1 1.2000000000000000e+01 \"Path \\\"A\\\"\\\\next\\n\\r\\t\"\n";
    check(output.str() == expected, "attribute rows use exact version-1 grammar");
    check(std::count(expected.begin(), expected.end(), '\n') == 3,
          "escaped label line breaks remain inside one physical row");

    std::istringstream typed_input(output.str());
    const auto color = line_color<real>::tag_read(typed_input);
    const auto font = line_font<real>::tag_read(typed_input);
    const auto label = text_label<real>::tag_read(typed_input);
    check(color && color->tag == 42 && color->entity->red == 1.0 &&
              color->entity->alpha == 0.25, "typed color decode");
    check(font && font->tag == 42 && font->entity->width == 1.5 &&
              font->entity->on == 4.0 && font->entity->off == 2.0,
          "typed line font decode");
    check(label && label->tag == 42 && label->entity->size == 12.0 &&
              label->entity->value == "Path \"A\"\\next\n\r\t",
          "typed label decode preserves every supported escape");

    check_scalar_round_trip<float>();
    check_scalar_round_trip<double>();
    check_scalar_round_trip<long double>();

    std::stringstream defaults;
    line_color<real>{}.tag_write(0, defaults);
    line_font<real>{}.tag_write(0, defaults);
    text_label<real>{}.tag_write(0, defaults);
    const auto default_color = line_color<real>::tag_read(defaults);
    const auto default_font = line_font<real>::tag_read(defaults);
    const auto default_text = text_label<real>::tag_read(defaults);
    check(default_color && default_color->entity->red == 0 &&
              default_color->entity->alpha == 1 && default_font &&
              default_font->entity->width == 1 &&
              default_font->entity->on == 0 && default_font->entity->off == 0 &&
              default_text && default_text->entity->size == 12 &&
              default_text->entity->value.empty(),
          "default opaque black, solid line and empty text round trip");

    std::stringstream mixed;
    point3<real>{1, 2, 3}.tag_write(42, mixed);
    vector3<real>{0, 1, 0}.tag_write(42, mixed);
    sphere3<real>{{0, 0, 0}, 1}.tag_write(42, mixed);
    make_line({0, 0, 0}, {1, 1, 1}).tag_write(42, mixed);
    mixed << output.str();
    for (std::size_t index = 0; index < 7; ++index) {
        const auto record = nurbspath::tag_read_record<real>(mixed);
        check(record && record->tag == 42 && record->value.index() == index &&
                  std::visit([](const auto& value) { return bool(value); },
                             record->value),
              "mixed reader preserves order, repeated tags and concrete types");
    }
    check(!nurbspath::tag_read_record<real>(mixed) && mixed.eof() && !mixed.bad(),
          "mixed reader reports clean EOF");

    std::istringstream final_row("9 text v1 3.5 \"hello\"");
    const auto final_text = text_label<real>::tag_read(final_row);
    check(final_text && final_text->tag == 9 &&
              final_text->entity->value == "hello" && !final_row.fail(),
          "text reader accepts final row without newline");
    std::istringstream crlf("9 text v1 3.5 \"hello\" \t\r\n");
    check(bool(text_label<real>::tag_read(crlf)),
          "text reader accepts CRLF and trailing whitespace");
    std::stringstream international;
    const std::string unicode = "M\xc3\xa4tpunkt \xe2\x86\x92";
    text_label<real>{10, unicode}.tag_write(10, international);
    const auto unicode_text = text_label<real>::tag_read(international);
    check(unicode_text && unicode_text->entity->value == unicode,
          "non-ASCII label bytes are preserved");

    for (const std::string_view row : {
             "1 linecolor v1 -0.1 0 0 1\n", "1 linecolor v1 0 1.1 0 1\n",
             "1 linecolor v1 0 0 nan 1\n", "1 linecolor v1 0 0 0 1.1\n",
             "1 linecolor v1 0 0 0\n", "1 linecolor v1 0 0 0 1 extra\n",
             "1 linefont v1 0 0 0\n", "1 linefont v1 inf 0 0\n",
             "1 linefont v1 1 -1 1\n", "1 linefont v1 1 0 1\n",
             "1 linefont v1 1 1 0\n", "1 linefont v1 1 2\n",
             "1 text v1 0 \"hello\"\n", "1 text v1 nan \"hello\"\n",
             "1 text v1 1 unquoted\n", "1 text v1 1 \"unterminated\n",
             "1 text v1 1 \"bad\\q\"\n", "1 text v1 1 \"tail\\\n",
             "1 text v1 1 \"extra\" token\n", "1 text v1 1 \"raw\ttab\"\n",
             "1 text v1 1 \"raw\rcarriage\"\n", "1 text v1 1 \"\x7f\"\n",
             "1 text v2 1 \"hello\"\n", "1 Text v1 1 \"hello\"\n"}) {
        check_invalid(row);
    }
    std::istringstream isolated("1 text v1 1 \"bad\\q\"\n2 point3 v1 1 2 3\n");
    check(!nurbspath::tag_read_record<real>(isolated) && isolated.fail(),
          "malformed attribute reports failure");
    isolated.clear();
    const auto next = nurbspath::tag_read_record<real>(isolated);
    check(next && next->tag == 2 && next->value.index() == 0,
          "malformed attribute consumes only its physical row");
    std::istringstream wrong_type("1 text v1 1 \"label\"\n");
    check(!line_color<real>::tag_read(wrong_type) && wrong_type.fail(),
          "typed attribute reader rejects mismatched types");
    std::istringstream legacy("1 linecolor v1 0 0 0 1\n");
    check(!nurbspath::tag_read<real>(legacy) && legacy.fail(),
          "existing geometry-only reader retains its strict type contract");

    const real infinity = std::numeric_limits<real>::infinity();
    check_invalid_write(line_color<real>{infinity, 0, 0, 1}, "nonfinite color");
    check_invalid_write(line_color<real>{0, 0, 0, -1}, "invalid alpha");
    check_invalid_write(line_font<real>{-1, 0, 0}, "negative line width");
    check_invalid_write(line_font<real>{1, 0, 1}, "one-sided dash pattern");
    check_invalid_write(line_font<real>{1, infinity, 1}, "nonfinite dash length");
    check_invalid_write(text_label<real>{infinity, "label"}, "nonfinite size");
    check_invalid_write(text_label<real>{-1, "label"}, "negative size");
    check_invalid_write(text_label<real>{1, std::string("a\0b", 3)}, "NUL byte");
    check_invalid_write(text_label<real>{1, "\x7f"}, "DEL byte");

    return finish("23_test_tagged_attributes");
}
