# Tagged geometry, presentation, and advisory data format

`nurbspath` provides a line-oriented text format for storing `point3`,
`vector3`, `sphere3`, and `nurbs_spline3` entities, plus optional line color,
line font, and text records, together with an application-defined
`std::size_t` tag. A tag can identify a time step, frame, sample, curve, or any
other application grouping. Tags do not need to be unique: several consecutive
or nonconsecutive rows may use the same tag. Optional output-only application
identity and positioned warning, information, and error messages can accompany
these rows. Every reader may ignore these advisory records.

The format is intended for sequential interchange. It has no required file
header, index, uniqueness constraint, or required row ordering. A file is
simply zero or more geometry, presentation, or advisory rows. An optional
untagged application row typically appears first but may appear anywhere.

The [illustrated PDF manual](https://github.com/magse/NURBSpath/blob/main/manual/nurbspath-manual.pdf) includes worked
examples of tagged geometry, presentation settings, and advisory output.

## Version 1 row grammar

Geometry and presentation version 1 rows have this prefix and one type-specific
payload:

```text
<tag> <type> v1 <payload>\n
```

The canonical writer separates tokens with one ASCII space and terminates each
row with a newline. The seven case-sensitive type tokens are `point3`,
`vector3`, `sphere3`, `spline3`, `linecolor`, `linefont`, and `text`. The
`spline3` storage token represents the
C++ type `nurbspath::nurbs_spline3<REAL>`. The distinct array-backed
`nurbspath::nurbs_arr_spline3<REAL>` type has no additional token or record
layout; explicitly convert it to an ordinary spline before writing tagged data.

`<tag>` is an unsigned base-10 integer that must fit in `std::size_t` on the
reading system. Tags are therefore textually portable, but a value produced on
a system with a wider `std::size_t` can be rejected on a narrower system.

Readers accept whitespace between fields and parse floating-point values with
the classic C locale. Writers always produce the canonical scientific form
described below. Empty rows, comments, unsupported version tokens, unknown
record types, missing fields, and surplus non-whitespace fields are not valid
for geometry and presentation records. Advisory application and diagnostic rows
have the separate grammars below; built-in readers skip those rows without
validating their payload or version.

### `point3`

```text
<tag> point3 v1 <x> <y> <z>
```

The three finite floating-point fields are the world-space X, Y, and Z
coordinates.

### `vector3`

```text
<tag> vector3 v1 <x> <y> <z>
```

The three finite floating-point fields are the X, Y, and Z components.

### `sphere3`

```text
<tag> sphere3 v1 <center_x> <center_y> <center_z> <radius>
```

The center coordinates and radius must be finite, and the radius must be
strictly positive.

### `spline3`

```text
<tag> spline3 v1 <degree> <closed> <tolerance> <control_count> <knot_count> \
    (<x> <y> <z> <weight>){control_count} (<knot>){knot_count}
```

The notation in parentheses describes repeated fields; the actual record is
still one physical row and contains no parentheses or braces. Its fields are,
in order:

1. `degree`: the unsigned polynomial degree.
2. `closed`: exactly `0` for an open spline or `1` for a closed spline.
3. `tolerance`: the finite, strictly positive definition and parameter
   tolerance.
4. `control_count`: the unsigned number of control points and weights.
5. `knot_count`: the unsigned number of knots.
6. `control_count` groups of control-point X, Y, and Z followed immediately by
   that control point's weight.
7. `knot_count` knot values.

The payload maps to the ordinary spline's `get_control_points()`,
`get_weights()`, `get_knots()`, `degree()`, `is_closed()`, and `tolerance()`
values.
`control_count` and `knot_count` are derived framing values. The on-disk
control-point groups interleave each point's coordinates with its weight; this
layout is independent of the flattened in-memory scalar order used by
`nurbs_arr_spline3`.

All control coordinates, weights, knots, and the tolerance must be finite.
Weights must be strictly positive, knots must be nondecreasing, and
`knot_count` must equal `control_count + degree + 1`, the value returned by
`nurbspath::nurbs_knot_count(degree, control_count)`. The
`nurbs_spline3` constructor validation also applies: the degree must be
positive and below the control-point count, and a row marked closed must
evaluate to coincident active-domain endpoints within the spline's closure
tolerance.

This layout stores every ordinary spline field needed to preserve the control
points, paired weights, knot vector, degree, closure state, tolerance, and
native active `s` domain. Typed and heterogeneous reads allocate
`nurbs_spline3<REAL>`. Callers that need array-backed storage can explicitly
convert the decoded ordinary spline; converting an array-backed spline to the
ordinary type before writing preserves the same logical fields.

### `linecolor`

```text
<tag> linecolor v1 <red> <green> <blue> <alpha>
```

The C++ type is `nurbspath::line_color<REAL>`, an aggregate with public
`red`, `green`, `blue`, and `alpha` fields. Each channel must be finite and in
`[0, 1]`. RGB channels are normalized color components; alpha is opacity, with
`0` transparent and `1` opaque. The default value is opaque black
`{0, 0, 0, 1}`. The format does not mandate a color-space conversion.

### `linefont`

```text
<tag> linefont v1 <width> <on> <off>
```

The C++ type is `nurbspath::line_font<REAL>`, an aggregate with public
`width`, `on`, and `off` fields. Width must be finite and strictly positive.
The dash lengths must be finite and nonnegative: `(on, off) = (0, 0)` means a
solid line, and a dashed line requires both lengths to be strictly positive.
A pair with exactly one zero is invalid. The default is `{1, 0, 0}`.

Width and dash-on and dash-off lengths use display units selected by the
consumer. The row does not prescribe pixel units, world units, or a scale
conversion.

### `text`

```text
<tag> text v1 <size> "text"
```

The C++ type is `nurbspath::text_label<REAL>`, an aggregate with public `size`
and `value` fields. Size must be finite and strictly positive and uses the
consumer's display units. The default is `{12, ""}`. The quoted string is
required even when empty. It can contain spaces, and its accepted escapes are:

| Encoded bytes | Decoded value |
|---|---|
| `\\` | Backslash |
| `\"` | Double quote |
| `\n` | Newline |
| `\r` | Carriage return |
| `\t` | Tab |

The writer emits these escapes, so every label remains one physical row.
Newline, carriage-return, and tab characters must use their escape sequences
inside the quoted row; raw ASCII control bytes in that row are invalid.
Unknown escapes, an unterminated string, or surplus fields are also invalid.
Decoded label values reject byte values `0` through `31` except `9`, `10`,
and `13`, plus `127`. Bytes at least `128` are preserved without UTF-8
validation. The format contains no text-position, orientation, or font-family
field; the consumer places and renders the label.

### Optional presentation and tag association

Presentation records are optional independent rows. For example, a consumer
can associate `linecolor`, `linefont`, and `text` rows with a 1D spline using
the spline's tag. They are not nested inside geometry, do not establish a
persistent drawing state, and do not require adjacency or a particular row
order. The library does not resolve duplicate tags or choose a policy for
repeated styles. Consumers define grouping, omitted-style defaults, and label
placement. These rows do not automatically alter geometry or SVG output.

## Optional application and diagnostic output

These output-only records are advisory. Readers are not required to interpret,
store, or display them. They do not change geometry, presentation defaults, or
SVG drawing state and do not participate in either input variant.

### `application`

```text
application v1 <name> <version>
```

The C++ type is `nurbspath::application_info`, an aggregate with public `name`
and `version` strings. It has no tag. Both strings must be nonempty unquoted
tokens. The writer rejects bytes `0` through `32` and byte `127`; bytes at
least `128` are preserved without UTF-8 validation. Thus ASCII spaces,
whitespace, and control characters are unavailable in the name and version.
The application may use a token such as `path_exporter` for its name and `1.0`
for its version. The strings default to empty and must be populated before
writing. An application row may appear anywhere, including more than once;
the consumer defines its meaning and any policy for repeated rows.

### `warning`, `information`, and `error`

```text
<tag> warning v1 <x> <y> <z> <size> "text"
<tag> information v1 <x> <y> <z> <size> "text"
<tag> error v1 <x> <y> <z> <size> "text"
```

The C++ type is `nurbspath::diagnostic_message3<REAL>`, an aggregate with public
`level`, `position`, `size`, and `value` fields. `level` is one of
`diagnostic_level::warning`, `diagnostic_level::information`, or
`diagnostic_level::error`; the default is `information`. `position` is a
`point3<REAL>` in the 3D Cartesian world, defaulting to the origin. All three
coordinates must be finite. `size` must be finite and strictly positive in
consumer-selected display units and defaults to `12`. `value` defaults to an
empty string and follows the quoted-text and escape rules of `text`, including
preserving bytes at least `128` without UTF-8 validation. An invalid enum value
is rejected before writing.

The tag provides application-defined association with other rows. Diagnostics
have their own 3D position rather than relying on the tag to locate text. The
consumer chooses whether and how to render that position and message. Writing
an `error` row does not itself throw, set stream failure, or terminate export;
only invalid fields or ordinary output errors affect writing.

The following preview uses scientific notation rounded to four digits after
the decimal point:

```text
application v1 path_exporter 1.0
42 warning v1 1.0000e+00 2.0000e+00 3.0000e+00 1.2000e+01 "Small clearance"
42 information v1 2.0000e+00 1.0000e+00 0.0000e+00 1.0000e+01 "Path complete"
43 error v1 3.0000e+00 0.0000e+00 1.0000e+00 1.4000e+01 "No intersection"
```

Writers retain the full round-trip precision specified below.
Built-in typed, geometry-only, and mixed-record readers recognize advisory rows
by a first token of `application`, or a valid unsigned `std::size_t` tag
followed by `warning`, `information`, or `error`. They skip the complete row
without validating the version or payload. Application rows require no tag;
diagnostic rows with malformed or out-of-range tags still fail. This permits
readers to ignore annotations they do not implement, including future versions
or malformed advisory payloads. Other unrecognized records retain the strict
failure behavior described below.

## Canonical writer output

Each geometry and presentation type and `diagnostic_message3` exposes:

```cpp
std::ostream& tag_write(std::size_t tag, std::ostream& output) const;
```

`application_info` instead exposes `std::ostream& write(std::ostream& output)
const`, without a tag argument. Every writer produces exactly one
newline-terminated row at the stream's current put position. It does not
explicitly request a flush; normal stream policy, including `unitbuf`, still
applies. Integer fields use base 10. Floating-point
fields use the classic C locale, lowercase scientific notation, and
`std::numeric_limits<REAL>::max_digits10 - 1` digits after the decimal point.
That produces `max_digits10` significant digits, sufficient to recover every
finite value of the same `REAL` type. The caller's stream flags, precision, and
locale do not affect the row and are not changed by the operation.

`point3`, `vector3`, and `sphere3` reject non-finite data with
`std::invalid_argument` before writing. The presentation types validate their
public fields before writing and throw `std::invalid_argument` for the invalid
values described above. The application and diagnostic types also validate all
public fields and throw `std::invalid_argument` before writing invalid data.
A valid `nurbs_spline3` already has finite stored fields by construction.
Output errors follow the destination stream's state and exception mask.

The following rows illustrate the format in a preview rounded to four digits
after the decimal point. Actual canonical output retains the full precision
described above:

```text
application v1 path_exporter 1.0
7 point3 v1 1.0000e+00 -2.5000e+00 3.0000e+00
7 vector3 v1 0.0000e+00 1.0000e+00 0.0000e+00
7 sphere3 v1 1.0000e+00 2.0000e+00 3.0000e+00 4.0000e+00
8 spline3 v1 1 0 5.0000e-01 2 4 0.0000e+00 0.0000e+00 0.0000e+00 1.0000e+00 1.0000e+00 2.0000e+00 3.0000e+00 1.0000e+00 0.0000e+00 0.0000e+00 1.0000e+00 1.0000e+00
8 linecolor v1 2.5000e-01 5.0000e-01 1.0000e+00 7.5000e-01
8 linefont v1 1.5000e+00 4.0000e+00 2.0000e+00
8 text v1 1.2000e+01 "Path \"A\"\nnext pass"
8 warning v1 1.0000e+00 2.0000e+00 3.0000e+00 1.2000e+01 "Small clearance"
8 information v1 2.0000e+00 1.0000e+00 0.0000e+00 1.0000e+01 "Path complete"
9 error v1 3.0000e+00 0.0000e+00 1.0000e+00 1.4000e+01 "No intersection"
```

The three geometry rows with tag `7` can describe entities at one time step.
The rows with tag `8` describe a spline, optional presentation data, and
advisory diagnostics a consumer can associate with it. The untagged first row
identifies the writing application. Built-in mixed readers return the seven
geometry and presentation records and skip the four advisory rows.

## Writing rows

The tagged member functions are available from each entity's own header and
from the umbrella header. Include `nurbspath/tagged_attributes.hpp` directly
when only the presentation types are needed, or
`nurbspath/tagged_annotations.hpp` for application and diagnostic output:

```cpp
#include <nurbspath/nurbspath.hpp>

#include <ostream>

void write_frame(std::ostream& output) {
    constexpr std::size_t time_step = 7;

    const nurbspath::point3<double> position{1.0, -2.5, 3.0};
    const nurbspath::vector3<double> velocity{0.0, 1.0, 0.0};
    const nurbspath::sphere3<double> boundary{{1.0, 2.0, 3.0}, 4.0};

    position.tag_write(time_step, output);
    velocity.tag_write(time_step, output);
    boundary.tag_write(time_step, output);
}
```

Repeated calls produce independent rows at the stream's current put position.
The library does not combine entities that share a tag.

The following complete function writes a curve, two optional line settings,
and an escaped label, followed by a separately tagged point and label:

```cpp
#include <nurbspath/config.hpp>
#include <nurbspath/nurbspath.hpp>

#include <ostream>

void write_styled_path(std::ostream& output) {
    const nurbspath::nurbs_spline3<double> path{
        nurbspath::point3<double>{0.0, 0.0, 0.0},
        nurbspath::point3<double>{2.0, 1.0, 0.0}, 2, 1};
    path.tag_write(42, output);
    nurbspath::line_color<double>{0.25, 0.5, 1.0, 0.75}
        .tag_write(42, output);
    nurbspath::line_font<double>{1.5, 4.0, 2.0}.tag_write(42, output);
    nurbspath::text_label<double>{12.0, "Path \"A\"\nnext pass"}
        .tag_write(42, output);

    nurbspath::point3<double>{2.0, 1.0, 0.0}.tag_write(43, output);
    nurbspath::text_label<double>{10.0, "End\\marker\tready"}
        .tag_write(43, output);
}
```

This complete function adds application identity and each diagnostic level:

```cpp
#include <nurbspath/config.hpp>
#include <nurbspath/tagged_annotations.hpp>

#include <ostream>

void write_annotations(std::ostream& output) {
    nurbspath::application_info{"path_exporter", "1.0"}.write(output);
    nurbspath::diagnostic_message3<double>{
        nurbspath::diagnostic_level::warning, {1.0, 2.0, 3.0},
        12.0, "Small clearance"}.tag_write(42, output);
    nurbspath::diagnostic_message3<double>{
        nurbspath::diagnostic_level::information, {2.0, 1.0, 0.0},
        10.0, "Path complete"}.tag_write(42, output);
    nurbspath::diagnostic_message3<double>{
        nurbspath::diagnostic_level::error, {3.0, 0.0, 1.0},
        14.0, "No intersection"}.tag_write(43, output);
}
```

Build and run `nurbspath_tagged_example` for the complete output-and-input
demonstration in `examples/tagged_usage.cpp`. The example prints a scientific
console preview with four digits after the decimal point and reads back the
full-precision buffered records. See [INSTALL.md](INSTALL.md) for the commands.

## Reading a known record type

Every supported geometry and presentation type provides a static `tag_read`
member. Advisory types are output-only and have no static reader. Before
reading its required type, every typed reader skips leading application and
diagnostic rows. Its result contains
the preserved tag and a non-null `std::shared_ptr` to the decoded concrete
entity:

```cpp
namespace nurbspath {

template <typename ENTITY>
struct tagged_read_result {
    std::size_t tag;
    std::shared_ptr<ENTITY> entity;
};

} // namespace nurbspath
```

For example:

```cpp
#include <nurbspath/point3.hpp>

#include <istream>
#include <stdexcept>

nurbspath::tagged_read_result<nurbspath::point3<double>>
read_point(std::istream& input) {
    auto result = nurbspath::point3<double>::tag_read(input);
    if (!result) {
        throw std::runtime_error("no valid tagged point3 row");
    }
    return *result;
}
```

A typed reader requires its matching type token. For example,
`point3<double>::tag_read` rejects a `vector3` row rather than converting or
skipping it. Similarly, `line_color<double>::tag_read`,
`line_font<double>::tag_read`, and `text_label<double>::tag_read` accept only
`linecolor`, `linefont`, and `text` respectively:

```cpp
auto color = nurbspath::line_color<double>::tag_read(input);
// On success: color->tag and color->entity->red/green/blue/alpha.
```

## Reading mixed entity types

Include `nurbspath/tagged_serialization.hpp`, or the umbrella
`nurbspath/nurbspath.hpp`, to use the general reader:

```cpp
namespace nurbspath {

template <std::floating_point REAL>
using entity3_ptr = std::variant<
    std::shared_ptr<point3<REAL>>,
    std::shared_ptr<vector3<REAL>>,
    std::shared_ptr<sphere3<REAL>>,
    std::shared_ptr<nurbs_spline3<REAL>>>;

template <std::floating_point REAL>
struct tagged_entity3 {
    std::size_t tag;
    entity3_ptr<REAL> entity;
};

} // namespace nurbspath
```

`nurbspath::tag_read<REAL>(input)` skips leading advisory rows, then reads at
most one geometry row, chooses the concrete type from its type token, allocates
it with shared ownership, and returns
`std::optional<tagged_entity3<REAL>>`:

```cpp
#include <nurbspath/tagged_serialization.hpp>

#include <istream>
#include <variant>

void read_all(std::istream& input) {
    while (auto record = nurbspath::tag_read<double>(input)) {
        std::visit(
            [tag = record->tag](const auto& entity) {
                // Process `*entity` for this time step. The pointer is non-null.
                (void)tag;
                (void)entity;
            },
            record->entity);
    }
}
```

No common geometry base class or implicit 2D/3D conversion is introduced; the
variant keeps the concrete 3D type explicit.

## Reading mixed geometry and presentation records

Include `nurbspath/tagged_serialization.hpp`, or the umbrella header, and use
`nurbspath::tag_read_record<REAL>(input)` for a stream containing any of the
seven geometry and presentation types, skipping any application and diagnostic
rows before the next record:

```cpp
namespace nurbspath {

template <std::floating_point REAL>
using tagged_record_ptr = std::variant<
    std::shared_ptr<point3<REAL>>,
    std::shared_ptr<vector3<REAL>>,
    std::shared_ptr<sphere3<REAL>>,
    std::shared_ptr<nurbs_spline3<REAL>>,
    std::shared_ptr<line_color<REAL>>,
    std::shared_ptr<line_font<REAL>>,
    std::shared_ptr<text_label<REAL>>>;

template <std::floating_point REAL>
struct tagged_record {
    std::size_t tag;
    tagged_record_ptr<REAL> value;
};

} // namespace nurbspath
```

The result is `std::optional<tagged_record<REAL>>`. A successful result owns a
non-null pointer to its concrete geometry or presentation value:

```cpp
#include <nurbspath/config.hpp>
#include <nurbspath/tagged_serialization.hpp>

#include <istream>
#include <variant>

void read_records(std::istream& input) {
    while (auto record = nurbspath::tag_read_record<double>(input)) {
        std::visit(
            [tag = record->tag](const auto& value) {
                // Collect *value under tag using the application's policy.
                (void)tag;
                (void)value;
            },
            record->value);
    }
}
```

The existing `tag_read`, `tagged_entity3`, and `entity3_ptr` APIs retain exactly
their four geometry alternatives. They reject presentation rows with `failbit`;
use `tag_read_record` when presentation rows may appear in the input. Neither
variant gains an application or diagnostic alternative; both reader families
skip these advisory rows.

## Reader and failure semantics

Typed, geometry-only, and mixed-record readers first consume any leading
application and diagnostic rows without validating their payloads or versions.
Diagnostic rows still require a valid tag as described above. Each call then
consumes at most one complete geometry or presentation row,
returning `std::nullopt` when no such row can be read at end of input or when
that row is malformed. A successful result owns a fully validated geometry or
presentation value; partially parsed records are never returned. Advisory rows
may be interleaved anywhere in a stream. A stream containing only advisory
rows ends with the normal `std::getline` EOF/failure state and no result.

Malformed geometry and presentation rows, and diagnostic rows with invalid
tags, set `failbit` on the source stream. Failures include a bad tag, wrong or
unknown type, unsupported version, missing or extra fields,
non-finite or out-of-range values, invalid counts, an invalid spline
definition, invalid line settings or text escaping, and a type mismatch in an
individual reader. The complete physical row has already been consumed when
such a parsing failure is reported. Stream
exceptions remain governed by the source stream's exception mask, and
allocation failures such as `std::bad_alloc` propagate normally.

End-of-input follows the normal `std::getline` stream behavior: a reader
returns `std::nullopt`, and the stream reports its EOF/failure state. A final
valid row is accepted even when it is not followed by a newline. Because both
EOF and malformed input produce `std::nullopt`, applications that need to
diagnose the stopping reason should also inspect the stream state. A reader
only searches past advisory rows; it never skips a malformed geometry or
presentation row to find a valid one. After clearing a recoverable `failbit`,
the next call starts at the following physical row.

For geometry and presentation records, the reader accepts ordinary
classic-locale floating-point input, including decimal or scientific spellings,
but writers always emit the canonical scientific representation. `NaN` and
infinities are rejected. Trailing whitespace and ordinary CRLF line endings
are accepted; trailing non-whitespace tokens are not.

Readers buffer one complete physical row before decoding it, and constructing
a spline performs work governed by its stored degree and control count. The
format deliberately imposes no application-specific row-size, degree, or
entity-count limit. Applications processing untrusted data should therefore
bound input line lengths and accepted spline complexity before passing rows to
these readers.

## Compatibility and evolution

Tagged rows are separate from the existing coordinate stream operators,
CSV/TSV helpers, and native binary `read`/`write` members. Those formats have no
tag or entity discriminator and are not interchangeable with this format.

The `v1` token identifies the row schema. For geometry and presentation
records, version 1 readers reject any other version instead of guessing its
field layout. A future schema can therefore use a new version token while
leaving the version 1 grammar unambiguous. Advisory application and diagnostic
rows are skipped without version validation, so they can evolve independently
of readers that do not interpret them.
