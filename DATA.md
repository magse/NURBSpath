# Tagged 3D data format

`nurbspath` provides a line-oriented text format for storing `point3`,
`vector3`, `sphere3`, and `nurbs_spline3` entities together with an
application-defined `std::size_t` tag. A tag can identify a time step, frame,
sample, or any other application grouping. Tags do not need to be unique:
several consecutive or nonconsecutive rows may use the same tag.

The format is intended for sequential interchange. It has no file header,
index, uniqueness constraint, or required row ordering. A file is simply zero
or more tagged rows.

## Version 1 row grammar

Every version 1 row has this prefix and one entity-specific payload:

```text
<tag> <type> v1 <payload>\n
```

The canonical writer separates tokens with one ASCII space and terminates each
row with a newline. The four case-sensitive type tokens are `point3`,
`vector3`, `sphere3`, and `spline3`. The `spline3` storage token represents the
C++ type `nurbspath::nurbs_spline3<REAL>`.

`<tag>` is an unsigned base-10 integer that must fit in `std::size_t` on the
reading system. Tags are therefore textually portable, but a value produced on
a system with a wider `std::size_t` can be rejected on a narrower system.

Readers accept whitespace between fields and parse floating-point values with
the classic C locale. Writers always produce the canonical scientific form
described below. Empty rows, comments, unsupported version tokens, unknown
entity types, missing fields, and surplus non-whitespace fields are not valid.

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

All control coordinates, weights, knots, and the tolerance must be finite.
Weights must be strictly positive, knots must be nondecreasing, and
`knot_count` must equal `control_count + degree + 1`. The ordinary
`nurbs_spline3` constructor validation also applies: the degree must be
positive and below the control-point count, and a row marked closed must
evaluate to coincident active-domain endpoints within the spline's closure
tolerance.

This layout stores the complete spline definition. Reading it preserves the
control points, paired weights, knot vector, degree, closure state, tolerance,
and native active `s` domain.

## Canonical writer output

Each entity exposes:

```cpp
std::ostream& tag_write(std::size_t tag, std::ostream& output) const;
```

`tag_write` writes exactly one newline-terminated row at the stream's current
put position. It does not explicitly request a flush; normal stream policy,
including `unitbuf`, still applies. Integer fields use base 10. Floating-point
fields use the classic C locale, lowercase scientific notation, and
`std::numeric_limits<REAL>::max_digits10 - 1` digits after the decimal point.
That produces `max_digits10` significant digits, sufficient to recover every
finite value of the same `REAL` type. The caller's stream flags, precision, and
locale do not affect the row and are not changed by the operation.

`point3`, `vector3`, and `sphere3` reject non-finite data with
`std::invalid_argument` before writing. A valid `nurbs_spline3` already has
finite definition fields by construction. Output errors follow the destination
stream's state and exception mask.

For `REAL = double`, canonical rows can look like this:

```text
7 point3 v1 1.0000000000000000e+00 -2.5000000000000000e+00 3.0000000000000000e+00
7 vector3 v1 0.0000000000000000e+00 1.0000000000000000e+00 0.0000000000000000e+00
7 sphere3 v1 1.0000000000000000e+00 2.0000000000000000e+00 3.0000000000000000e+00 4.0000000000000000e+00
8 spline3 v1 1 0 5.0000000000000000e-01 2 4 0.0000000000000000e+00 0.0000000000000000e+00 0.0000000000000000e+00 1.0000000000000000e+00 1.0000000000000000e+00 2.0000000000000000e+00 3.0000000000000000e+00 1.0000000000000000e+00 0.0000000000000000e+00 0.0000000000000000e+00 1.0000000000000000e+00 1.0000000000000000e+00
```

The first three rows deliberately reuse tag `7`, for example to describe three
entities at one time step.

## Writing rows

The tagged member functions are available from each entity's own header and
from the umbrella header:

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

## Reading a known entity type

Every supported entity provides a static `tag_read` member. Its result contains
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
skipping it.

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

`nurbspath::tag_read<REAL>(input)` reads one physical row, chooses the concrete
type from its type token, allocates it with shared ownership, and returns
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

## Reader and failure semantics

Both typed and general readers consume at most one complete physical row per
call and return `std::nullopt` when no row can be read at end of input or when
the consumed row is malformed. A successful result owns a fully validated
entity; partially parsed entities are never returned.

Malformed rows set `failbit` on the source stream. This includes a bad tag,
wrong or unknown type, unsupported version, missing or extra fields,
non-finite or out-of-range values, invalid counts, an invalid spline
definition, and a type mismatch in an individual reader. The complete physical
row has already been consumed when such a parsing failure is reported. Stream
exceptions remain governed by the source stream's exception mask, and
allocation failures such as `std::bad_alloc` propagate normally.

End-of-input follows the normal `std::getline` stream behavior: a reader
returns `std::nullopt`, and the stream reports its EOF/failure state. A final
valid row is accepted even when it is not followed by a newline. Because both
EOF and malformed input produce `std::nullopt`, applications that need to
diagnose the stopping reason should also inspect the stream state. A reader
never searches ahead for another valid row; after clearing a recoverable
`failbit`, the next call starts at the following physical row.

The reader accepts ordinary classic-locale floating-point input, including
decimal or scientific spellings, but writers always emit the canonical
scientific representation. `NaN` and infinities are rejected. Trailing
whitespace and ordinary CRLF line endings are accepted; trailing non-whitespace
tokens are not.

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

The `v1` token identifies the row schema. Version 1 readers reject any other
version instead of guessing its field layout. A future schema can therefore
use a new version token while leaving the version 1 grammar unambiguous.
