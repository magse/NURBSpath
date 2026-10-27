#include <nurbspath/config.hpp>
#include <nurbspath/nurbspath.hpp>

#include "test_support.hpp"

#include <algorithm>
#include <concepts>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

namespace {

using test_support::real;
using nurbspath::application_info;
using nurbspath::diagnostic_level;
using nurbspath::diagnostic_message3;

struct comma_decimal : std::numpunct<char> {
    char do_decimal_point() const override { return ','; }
};

template <typename WRITER>
void check_invalid_write(WRITER write, std::string_view message) {
    std::ostringstream output;
    output << "unchanged";
    bool rejected = false;
    try {
        write(output);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    test_support::check(rejected && output.str() == "unchanged", message);
}

void check_invalid_application(const application_info& application) {
    check_invalid_write(
        [&](std::ostream& output) { application.write(output); },
        "invalid application token is rejected before writing");
}

void check_invalid_diagnostic(const diagnostic_message3<real>& diagnostic) {
    check_invalid_write(
        [&](std::ostream& output) { diagnostic.tag_write(42, output); },
        "invalid diagnostic is rejected before writing");
}

template <std::floating_point REAL>
void check_scalar_output() {
    const REAL tiny = std::numeric_limits<REAL>::denorm_min();
    const diagnostic_message3<REAL> diagnostic{
        diagnostic_level::warning,
        {REAL(0.1), -std::numeric_limits<REAL>::max(),
         tiny > REAL(0) ? tiny : std::numeric_limits<REAL>::min()},
        std::numeric_limits<REAL>::min(), "precise"};
    std::ostringstream output;
    diagnostic.tag_write(8, output);
    std::istringstream input(output.str());
    input.imbue(std::locale::classic());
    std::size_t tag = 0;
    std::string level;
    std::string version;
    input >> tag >> level >> version;
    REAL x = 0;
    REAL y = 0;
    REAL z = 0;
    REAL size = 0;
    const bool parsed = nurbspath::detail::read_real_token(input, x) &&
        nurbspath::detail::read_real_token(input, y) &&
        nurbspath::detail::read_real_token(input, z) &&
        nurbspath::detail::read_real_token(input, size);
    test_support::check(parsed && tag == 8 && level == "warning" &&
        version == "v1" && x == diagnostic.position.x &&
        y == diagnostic.position.y && z == diagnostic.position.z &&
        size == diagnostic.size,
        "diagnostic scientific fields retain each scalar type's precision");
}

template <typename READER>
void check_reader_skips(
    std::string_view optional_row,
    std::string_view required_row,
    READER read,
    std::string_view message) {
    std::istringstream input{std::string(optional_row) + std::string(required_row)};
    const auto record = read(input);
    test_support::check(record && record->tag == 9 && !input.fail(), message);
}

void check_all_readers_skip(std::string_view row) {
    check_reader_skips(row, "9 point3 v1 1 2 3\n",
        [](std::istream& input) { return nurbspath::tag_read<real>(input); },
        "geometry dispatch skips advisory rows");
    check_reader_skips(row, "9 text v1 12 \"label\"\n",
        [](std::istream& input) { return nurbspath::tag_read_record<real>(input); },
        "mixed dispatch skips advisory rows");
    check_reader_skips(row, "9 point3 v1 1 2 3\n",
        [](std::istream& input) { return nurbspath::point3<real>::tag_read(input); },
        "typed point reader skips advisory rows");
    check_reader_skips(row, "9 vector3 v1 1 2 3\n",
        [](std::istream& input) { return nurbspath::vector3<real>::tag_read(input); },
        "typed vector reader skips advisory rows");
    check_reader_skips(row, "9 sphere3 v1 0 0 0 1\n",
        [](std::istream& input) { return nurbspath::sphere3<real>::tag_read(input); },
        "typed sphere reader skips advisory rows");
    check_reader_skips(row,
        "9 spline3 v1 1 0 1e-12 2 4 0 0 0 1 1 0 0 1 0 0 1 1\n",
        [](std::istream& input) { return nurbspath::nurbs_spline3<real>::tag_read(input); },
        "typed spline reader skips advisory rows");
    check_reader_skips(row, "9 linecolor v1 0 0 0 1\n",
        [](std::istream& input) { return nurbspath::line_color<real>::tag_read(input); },
        "typed line color reader skips advisory rows");
    check_reader_skips(row, "9 linefont v1 1 0 0\n",
        [](std::istream& input) { return nurbspath::line_font<real>::tag_read(input); },
        "typed line font reader skips advisory rows");
    check_reader_skips(row, "9 text v1 12 \"label\"\n",
        [](std::istream& input) { return nurbspath::text_label<real>::tag_read(input); },
        "typed text reader skips advisory rows");
}

} // namespace

int main() {
    using namespace test_support;

    static_assert(std::is_aggregate_v<application_info>);
    static_assert(std::is_aggregate_v<diagnostic_message3<real>>);
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
    const auto& application_stream = application_info{"path_tool", "2.3.4"}.write(output);
    const auto& diagnostic_stream = diagnostic_message3<real>{
        diagnostic_level::warning, {1, -2, 3.5}, 12,
        "Path \"A\"\\next\n\r\t"}.tag_write(42, output);
    diagnostic_message3<real>{diagnostic_level::information, {0, 0, 0}, 1, "ready"}
        .tag_write(7, output);
    diagnostic_message3<real>{diagnostic_level::error, {-1, 2, 0.5}, 2, "failed"}
        .tag_write(0, output);
    check(&application_stream == &output && &diagnostic_stream == &output,
          "advisory writers return their destination stream");
    check(output.flags() == flags && output.precision() == 2 &&
              output.width() == 8 && output.fill() == '_' && output.getloc() == locale,
          "advisory writers preserve caller formatting and locale");
    const std::string expected =
        "application v1 path_tool 2.3.4\n"
        "42 warning v1 1.0000000000000000e+00 -2.0000000000000000e+00 "
        "3.5000000000000000e+00 1.2000000000000000e+01 "
        "\"Path \\\"A\\\"\\\\next\\n\\r\\t\"\n"
        "7 information v1 0.0000000000000000e+00 0.0000000000000000e+00 "
        "0.0000000000000000e+00 1.0000000000000000e+00 \"ready\"\n"
        "0 error v1 -1.0000000000000000e+00 2.0000000000000000e+00 "
        "5.0000000000000000e-01 2.0000000000000000e+00 \"failed\"\n";
    const std::string encoded = output.str();
    check(encoded == expected, "advisory rows use the exact version-1 grammar");
    check(std::count(encoded.begin(), encoded.end(), '\n') == 4,
          "advisory writers escape line breaks into one physical row per record");
    std::ostringstream default_output;
    diagnostic_message3<real>{}.tag_write(0, default_output);
    check(default_output.str() ==
        "0 information v1 0.0000000000000000e+00 0.0000000000000000e+00 "
        "0.0000000000000000e+00 1.2000000000000000e+01 \"\"\n",
        "default diagnostic is empty information at the world origin with size 12");
    check_scalar_output<float>();
    check_scalar_output<double>();
    check_scalar_output<long double>();

    const std::string international = "M\xc3\xa4tpunkt\x80\xff";
    std::ostringstream international_output;
    application_info{international, international}.write(international_output);
    diagnostic_message3<real>{diagnostic_level::information, {}, 1, international}
        .tag_write(1, international_output);
    check(international_output.str().find(
              "application v1 " + international + ' ' + international + '\n') == 0 &&
              international_output.str().find('"' + international + '"') != std::string::npos,
          "advisory fields preserve non-ASCII bytes without UTF-8 validation");

    check_invalid_application({"", "1"});
    check_invalid_application({"tool", ""});
    for (int byte = 0; byte <= 32; ++byte) {
        const std::string token = std::string("a") + static_cast<char>(byte) + 'b';
        check_invalid_application({token, "1"});
        check_invalid_application({"tool", token});
    }
    check_invalid_application({"tool\x7f", "1"});
    check_invalid_application({"tool", "1\x7f"});
    const real infinity = std::numeric_limits<real>::infinity();
    const real nan = std::numeric_limits<real>::quiet_NaN();
    for (const real nonfinite : {infinity, -infinity, nan}) {
        check_invalid_diagnostic({diagnostic_level::warning, {nonfinite, 0, 0}, 1, "text"});
        check_invalid_diagnostic({diagnostic_level::warning, {0, nonfinite, 0}, 1, "text"});
        check_invalid_diagnostic({diagnostic_level::warning, {0, 0, nonfinite}, 1, "text"});
        check_invalid_diagnostic({diagnostic_level::warning, {}, nonfinite, "text"});
    }
    check_invalid_diagnostic({diagnostic_level::warning, {}, 0, "text"});
    check_invalid_diagnostic({diagnostic_level::warning, {}, -1, "text"});
    check_invalid_diagnostic({static_cast<diagnostic_level>(-1), {}, 1, "text"});
    check_invalid_diagnostic({static_cast<diagnostic_level>(3), {}, 1, "text"});
    for (int byte = 0; byte < 32; ++byte) {
        if (byte != '\n' && byte != '\r' && byte != '\t') {
            check_invalid_diagnostic({diagnostic_level::information, {}, 1,
                std::string("a") + static_cast<char>(byte) + 'b'});
        }
    }
    check_invalid_diagnostic({diagnostic_level::information, {}, 1, "\x7f"});

    for (const std::string_view row : {
             "application v1 tool 1\n", "application v99 unrecognized payload\n",
             "application\n", "application malformed\n",
             "1 warning v1 1 2 3 12 \"warning\"\n",
             "2 information v1 1 2 3 12 \"information\"\n",
             "3 error v1 1 2 3 12 \"error\"\n",
             "1 warning v99\n", "2 information\n", "3 error v1 garbage\n",
             "1 warning v1 nan inf 0 -1 \"bad\\q\" trailing\n",
             " \tapplication v1 tool 1 \r\n", " \t1 error v2 malformed\r\n"}) {
        check_all_readers_skip(row);
    }

    std::istringstream interleaved(
        "application v1 tool 1\n1 warning v2 junk\n9 point3 v1 1 2 3\n"
        "2 information\n9 linefont v1 1 0 0\n3 error v1 \"unterminated\n"
        "9 text v1 12 \"label\"");
    const auto point = nurbspath::tag_read_record<real>(interleaved);
    const auto font = nurbspath::tag_read_record<real>(interleaved);
    const auto text = nurbspath::tag_read_record<real>(interleaved);
    check(point && point->value.index() == 0 && font && font->value.index() == 5 &&
              text && text->value.index() == 6 && !interleaved.fail(),
          "advisory rows do not alter order or types of required records");
    check_reader_skips("application v1 tool 1\r\n1 warning v2 malformed\r\n",
        "9 point3 v1 1 2 3",
        [](std::istream& input) { return nurbspath::point3<real>::tag_read(input); },
        "advisory skipping accepts CRLF and required final row without newline");
    for (const std::string_view optional_only : {
             "application v1 tool 1\n", "1 warning v1 malformed",
             "application\r\n1 information v2\r\n2 error"}) {
        std::istringstream geometry_input{std::string(optional_only)};
        std::istringstream mixed_input{std::string(optional_only)};
        std::istringstream typed_input{std::string(optional_only)};
        check(!nurbspath::tag_read<real>(geometry_input) && geometry_input.eof() &&
                  !geometry_input.bad(), "geometry reader reaches EOF after advisory-only input");
        check(!nurbspath::tag_read_record<real>(mixed_input) && mixed_input.eof() &&
                  !mixed_input.bad(), "mixed reader reaches EOF after advisory-only input");
        check(!nurbspath::text_label<real>::tag_read(typed_input) && typed_input.eof() &&
                  !typed_input.bad(), "typed reader reaches EOF after advisory-only input");
    }

    for (const std::string_view required_invalid : {
             "\n", "1 unknown v1 payload\n", "1 point3 v2 1 2 3\n",
             "1 point3 v1 1 2\n", "1 text v1 1 \"bad\\q\"\n",
             "application_extra v1 tool 1\n", "1 Warning v1 junk\n",
             "warning v1 junk\n", "bad warning v1 junk\n"}) {
        std::istringstream input{
            "application v1 tool 1\n1 warning v9 ignored\n" +
            std::string(required_invalid) + "2 error v1 ignored\n9 point3 v1 1 2 3\n"};
        check(!nurbspath::tag_read_record<real>(input) && input.fail() && !input.eof(),
              "advisory skipping keeps malformed required rows strict");
        input.clear();
        const auto recovered = nurbspath::tag_read_record<real>(input);
        check(recovered && recovered->tag == 9 && recovered->value.index() == 0,
              "recovery consumes only the malformed required row before resuming skipping");
    }
    std::istringstream wrong_type("1 warning v1 ignored\n9 text v1 12 \"label\"\n");
    check(!nurbspath::point3<real>::tag_read(wrong_type) && wrong_type.fail(),
          "typed reader still rejects the next required record of another type");

    return finish("24_test_tagged_annotations");
}
