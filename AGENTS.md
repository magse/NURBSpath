# Contributor guidance for nurbspath

## Project intent

`nurbspath` is a small, dependency-free, header-only C++20 library for 2D and
3D NURBS paths and tolerance-aware numerical geometry. The 2D and 3D Cartesian
worlds are intentionally separate. Do not introduce implicit conversions or
coordinate-frame transforms between them; `project(plane3, entity2)` is the
only entity bridge.

## Required validation

After changing public headers or CMake files, run:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Run `./build/nurbspath_example` when changing interpolation, intersections,
distances, or the example itself. Preserve a warning-clean build under the
`-Wall -Wextra -Wpedantic` test target on Clang/GCC or `/W4 /permissive-` on
MSVC.

When changing `graphics.hpp`, also run `nurbspath_svg_example` from a temporary
working directory, validate all three generated files as XML, and visually
inspect flat 2D, orthographic, and perspective output. Do not write generated
SVGs into the repository root.

## Public style and compatibility

- Use `snake_case` for files, types, functions, variables, and data members.
- Geometry templates use the floating-point template parameter name `REAL` and
  constrain it with `std::floating_point`.
- Require C++20, CMake 3.20, and no external dependency.
- Keep the library header-only unless a future requirement explicitly changes
  that design.
- Add public headers to `include/nurbspath` and expose them from
  `include/nurbspath/nurbspath.hpp` when generally useful.
- Document every public class and function with Doxygen-compatible `@brief`,
  `@param`, `@return`, `@tparam`, and `@throws` entries where applicable.
- Keep implementation-only free helpers and support types in
  `nurbspath::detail`; private class members may remain private implementation.
- Keep geometry allocation helpers in `include/nurbspath/creators.hpp`. They
  mirror entity constructors and return `std::shared_ptr`, preserving the
  constructor's validation and exceptions. Keep the ordinary NURBS factory
  overloads available for both `std::vector` and `std::valarray` inputs, and
  mirror the array-backed spline constructors with corresponding factories.
- Prefer `[[nodiscard]]`, `const`, `constexpr`, and `noexcept` where their
  contracts are accurate. Do not add `noexcept` to code that can validate and
  throw.
- Document units, parameter domains, tolerances, exceptional cases, and
  numerical limitations beside the API.

## Version metadata

- Every public `.hpp` and project `.cpp` file must directly include
  `nurbspath/config.hpp`. Only `config.hpp` itself is exempt.
- Keep semantic library and Git repository macros in `config.hpp`. The
  source-tree copy is the no-CMake fallback; CMake must generate a refreshed
  build-tree copy from `cmake/config.hpp.in` and install that generated copy.
- Keep the version in `CMakeLists.txt`, the source fallback, config tests, and
  user documentation aligned. Reconfigure after a commit, tag, or worktree
  change before validating generated Git metadata.
- A repository without `HEAD` must configure successfully and report an
  `unborn` commit. Preserve explicit dirty-state and commit-availability
  macros.
- Keep `manual.hpp` as the umbrella-exposed extension point for future
  hand-authored functions under `nurbspath::manual`.

## Geometry contracts

- `point2`/`point3` represent positions; `vector2`/`vector3` represent offsets
  or directions. Preserve both semantic boundaries and the 2D/3D world boundary.
- Ray and spline parameters are named `s`. Sphere and plane surface parameters
  are named `u` and `v`; circle angle is named `u`.
- A ray is forward for `s >= 0` and does not normalize its direction.
- A spline operates on its active knot domain `[knots[degree],
  knots[control_point_count]]`.
- Closed splines have coincident active-domain endpoints. Open splines need not.
- Sphere `u` is longitude in `[0, 2*pi)` and `v` is latitude in
  `[-pi/2, pi/2]`.
- Plane basis directions must remain orthonormal, with the normal completing a
  right-handed local frame.
- Plane normal-distance construction uses Hessian form `n_hat dot x = d`, where
  `d` is signed world distance and its parameter origin is `d*n_hat`.
- Positive NURBS weights are required. Knot vectors are nondecreasing.
- There is no `plane2`. Embed a 2D entity only through an explicit `plane3`.
- Point/vector/ray projection uses the plane's orthonormal `(u,v)` frame.
  Circle projection produces a sphere from projected center and unchanged
  radius. Spline projection changes only control points and preserves weights,
  knots, degree, tolerance, closure state, and native `s`.
- SVG cameras are defined by world-space eyepoint and look-at points. Keep
  `vector2` and `vector3` unrenderable by themselves because they have no
  world-space origin.

## Numerical code

Keep reusable numerical algorithms in `include/nurbspath/utility.hpp`; do not
bury a general solver in a geometry class. Route intersection controls through
`numerical_settings<REAL>`.

Numerical intersection changes must test:

- no contact;
- ordinary crossing;
- endpoint contact;
- tangent contact without a sign change;
- parallel or coincident behavior where applicable;
- result ordering and duplicate suppression.

Queries accepting only 2D entities must calculate residuals, closest points,
and parameters entirely in the 2D world. Do not implement them by projecting
to 3D. The supported 2D surface queries are ray-circle and spline-circle
intersections plus point-circle and point-spline distances.

Distance changes must preserve tolerance snapping, closest world points, and
the associated `s` or `(u, v)` parameters. Sampling-based algorithms must state
that their coverage depends on `sample_count`; never imply an unconditional
continuous global guarantee.

## NURBS implementation

2D and 3D positions and derivatives are evaluated from nonzero B-spline basis
functions. First, second, and third rational derivatives use homogeneous
numerator/weight derivatives and the quotient rule. Do not replace these
analytic derivatives with finite differences.

Keep the definition storage of `nurbs_spline2` and `nurbs_spline3` private.
Their definition setters must retain the current degree, validate a complete
candidate before committing it, and refresh cached endpoints and any other
derived state. Failed updates leave the spline unchanged; use bulk setters for
correlated changes such as closed seams or knot rescaling. Keep cached
endpoints readable through member getters but derived and read-only; do not add
direct endpoint setters. Target endpoint changes by editing the control-point
definition.

`nurbs_arr_spline2` and `nurbs_arr_spline3` are distinct, non-derived
alternatives whose flattened scalar `std::valarray<REAL>` storage is public so
callers can copy or replace it directly. Because direct edits bypass checked
setters, never keep endpoint or other derived caches that can become stale.
Validate the current array shape and values before geometry operations or
conversion, and reject malformed storage without indexing out of bounds.
Reconstruct `point2` and `point3` values from their scalar coordinates; do not
reinterpret or alias scalar array elements as point objects. Conversions to and
from the corresponding ordinary spline stay within the same 2D or 3D world and
preserve control points, weights, knots, degree, tolerance, closure state, and
the native `s` domain.

`interpolate` and `adopt_to_points` perform global interpolation through the
provided samples. The supplied strictly increasing arc-length stations remain
the native `s` values; do not silently normalize them. Unit weights make the
result a valid non-rational subset of NURBS. Closed input has coincident first
and final positions at strictly increasing stations; open input may end
elsewhere. Both forms use averaged clamped knots.

## SVG graphics

Keep all SVG projection and entity drawing code in
`include/nurbspath/graphics.hpp`. Orthographic projection must use a uniform
screen scale, and perspective segments must be clipped at the near plane.
Preserve the diagnostic conventions documented in `README.md`: black points,
persistent unit XYZ world axes with colored labels and half-width lines,
pale red dashed infinite ray lines with solid positive-direction unit lines and
origin dots, pale blue sphere silhouettes with visible/hidden equators and
center dots, pale green plane squares with unit
normals, half-unit positive-u/positive-v lines, and origin dots, and
piecewise-linear splines with dashed control polygons, control-point dots, and
evaluated endpoint dots. The default plane square side is one quarter unit.
Plane-embedded 2D points, rays, circles, and splines use the same colors and
glyphs as their projected 3D forms and must be added as `add(plane, entity)`.
Use dash-dotted primary entity strokes and dotted auxiliary, hidden,
positive-direction, and control-polygon strokes for projected 2D geometry.
The separate `svg_canvas2` accepts native 2D points, rays, circles, and splines
only, performs no plane projection, and uses solid lines for every entity and
construction stroke. Preserve its unit red/green XY reference at the 2D origin.
Tests should check both projections, finite output coordinates, requested
tessellation counts, and invalid camera settings. The generated SVG must remain
self-contained and dependency-free.

## Tagged text persistence

Keep optional `linecolor`, `linefont`, and `text` version-1 records in
`include/nurbspath/tagged_attributes.hpp` and expose them from the umbrella
header. These records are independent metadata: tag association, omitted-style
defaults, and label placement belong to the consumer. They do not change
geometry or establish SVG drawing state. Preserve the four geometry-only
alternatives in `entity3_ptr`, `tagged_entity3`, and `tag_read`; use
`tagged_record_ptr`, `tagged_record`, and `tag_read_record` for streams mixing
geometry and presentation records.

Validate finite normalized RGBA channels in `[0, 1]`, strictly positive finite
line width and text size, and finite dash lengths that are either both zero
(solid) or both positive (dashed). Width, dash lengths, and text size use
consumer-selected display units. Text is quoted, with `\\`, `\"`, `\n`,
`\r`, and `\t` escapes; reject other ASCII control bytes and preserve bytes
at least 128 without UTF-8 validation. Keep one physical row per record and
round-trip scientific numeric output.

Keep output-only `application_info`, `diagnostic_level`, and
`diagnostic_message3<REAL>` in `include/nurbspath/tagged_annotations.hpp` and
expose them from the umbrella header. Application output is untagged:
`application v1 <name> <version>`. Name and version must be nonempty unquoted
tokens, rejecting bytes at most 32 and byte 127 while preserving bytes at least
128. Diagnostic output is `<tag> warning|information|error v1 <x> <y> <z>
<size> "text"` on one physical row. Validate finite 3D world positions, finite
strictly positive size in consumer-selected display units, valid diagnostic
levels, and the existing quoted-text rules. The default level is information,
the position is the 3D origin, size is 12, and text is empty. Readers need not
interpret or display these advisory records; they do not change geometry or SVG
state. Do not add static readers or variant alternatives for them.

Typed, geometry-only, and mixed readers skip leading or interleaved advisory
rows without validating their version or payload. Recognize `application` as
the first token, or a valid unsigned `std::size_t` tag followed by `warning`,
`information`, or `error`. Malformed or out-of-range diagnostic tags still fail.
After skipping such rows, consume at most one required record, preserving its
strict validation, complete-row isolation, and failbit behavior. EOF after only
advisory rows follows normal getline EOF semantics. Cover valid, invalid,
escaped, mixed, and advisory-skipping cases in tests and keep the runnable
tagged-data example and all format documentation current.

## Repository map

- `CMakeLists.txt`: build, test, documentation, install, and package export
- `include/nurbspath/`: public header-only implementation
- `tests/01_test_*.cpp` through `tests/24_test_*.cpp`: focused numbered tests
- `tests/test_support.hpp`: shared dependency-free test helpers
- `examples/basic_usage.cpp`: minimal runnable 3D integration example
- `examples/basic_usage_2d.cpp`: minimal runnable native 2D example
- `examples/svg_usage.cpp`: orthographic and perspective SVG example
- `examples/tagged_usage.cpp`: optional line settings, text, advisory output,
  and mixed tagged input
- `cmake/config.hpp.in`: CMake-refreshed version/Git metadata header
- `cmake/nurbspath-config.cmake.in`: installed package configuration
- `README.md`: API, coordinate, and numerical behavior
- `DATA.md`: versioned format for 3D geometry, presentation, and advisory records
- `INSTALL.md`: build, install, and consumption instructions
- [`manual/README.md`](manual/README.md): standalone illustrated
  [PDF manual](manual/nurbspath-manual.pdf), split LaTeX sources, TikZ reference
  data, and build instructions
- `.gitignore`: generated build, release, cache, and platform metadata rules

## Documentation

Keep `README.md` focused on library behavior and public API usage. Keep
`INSTALL.md` focused on requirements, CMake workflows, installation, and
consumption. Ensure commands are runnable from the stated directory, examples
use current public names, relative links resolve, and version claims match
`config.hpp`. Keep floating-point example output, generated SVG drawing
attributes, and documentation previews in scientific notation with at most
four digits after the decimal point.
Clearly label rounded tagged-data previews; preserve full round-trip precision
in the canonical tagged writers and data used for round-trip demonstrations.

When changing public API documentation or CMake documentation settings, run an
opt-in documentation build and keep it warning-free:

```sh
cmake -S . -B build-docs \
  -DNURBSPATH_BUILD_DOCUMENTATION=ON \
  -DNURBSPATH_BUILD_TESTS=OFF \
  -DNURBSPATH_BUILD_EXAMPLES=OFF
cmake --build build-docs --target docs
```

Do not commit generated build or release directories, editor metadata,
platform-specific IDE state, or files such as `.DS_Store`.
