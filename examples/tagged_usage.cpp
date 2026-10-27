#include <nurbspath/config.hpp>
#include <nurbspath/nurbspath.hpp>

#include "output_format.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <variant>

int main() {
    using real = double;
    constexpr std::size_t path_tag = 42;
    constexpr std::size_t point_tag = 43;
    const nurbspath::nurbs_spline3<real> path(
        {{0.0, 0.0, 0.0}, {1.0, 2.0, 0.0}},
        {1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, 1);

    std::stringstream rows;
    // Application and diagnostic rows are optional output annotations. Readers
    // may ignore them; built-in tagged readers skip these complete rows.
    nurbspath::application_info{
        "nurbspath_tagged_example", NURBSPATH_VERSION_STRING}.write(rows);
    path.tag_write(path_tag, rows);
    nurbspath::line_color<real>{0.2, 0.4, 0.8, 0.75}.tag_write(path_tag, rows);
    nurbspath::line_font<real>{1.5, 4.0, 2.0}.tag_write(path_tag, rows);
    nurbspath::text_label<real>{12.0, "Path \"A\"\nmeasured"}
        .tag_write(path_tag, rows);
    nurbspath::diagnostic_message3<real>{
        nurbspath::diagnostic_level::warning, {0.0, 0.0, 0.0}, 10.0,
        "Check the path start"}.tag_write(path_tag, rows);

    // Attributes are optional: a point can carry only its own text label.
    nurbspath::point3<real>{1.0, 2.0, 0.0}.tag_write(point_tag, rows);
    nurbspath::text_label<real>{10.0, "End point"}.tag_write(point_tag, rows);
    nurbspath::diagnostic_message3<real>{
        nurbspath::diagnostic_level::information, {1.0, 2.0, 0.0}, 10.0,
        "End point reached"}.tag_write(point_tag, rows);

    // A second curve illustrates the explicit solid-line convention.
    path.tag_write(44, rows);
    nurbspath::line_font<real>{2.0, 0.0, 0.0}.tag_write(44, rows);
    nurbspath::diagnostic_message3<real>{
        nurbspath::diagnostic_level::error, {0.5, 1.0, 0.0}, 12.0,
        "Example \"error\"\nannotation only"}.tag_write(44, rows);
    // Preview rounding affects display only; readers below use full precision.
    nurbspath_examples::write_tagged_preview(std::cout, rows.str());

    // Applications choose the display units, tag grouping, and label placement.
    // The eight geometry and presentation records are returned in order, while
    // application identification and all three diagnostic rows are ignored.
    std::size_t record_count = 0;
    while (const auto record = nurbspath::tag_read_record<real>(rows)) {
        std::visit([&record_count](const auto& value) {
            if (value) {
                ++record_count;
            }
        }, record->value);
    }
    if (record_count != 8 || !rows.eof() || rows.bad()) {
        std::cerr << "Tagged record round trip failed\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
