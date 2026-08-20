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
  mirror entity constructors and return `std::unique_ptr`, preserving the
  constructor's validation and exceptions. Keep the NURBS factory overloads
  available for both `std::vector` and `std::valarray` definitions.
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

## Repository map

- `CMakeLists.txt`: build, test, documentation, install, and package export
- `include/nurbspath/`: public header-only implementation
- `tests/01_test_*.cpp` through `tests/17_test_*.cpp`: focused numbered tests
- `tests/test_support.hpp`: shared dependency-free test helpers
- `examples/basic_usage.cpp`: minimal runnable 3D integration example
- `examples/basic_usage_2d.cpp`: minimal runnable native 2D example
- `examples/svg_usage.cpp`: orthographic and perspective SVG example
- `cmake/config.hpp.in`: CMake-refreshed version/Git metadata header
- `cmake/nurbspath-config.cmake.in`: installed package configuration
- `README.md`: API, coordinate, and numerical behavior
- `INSTALL.md`: build, install, and consumption instructions
- `.gitignore`: generated build, release, cache, and platform metadata rules

## Documentation

Keep `README.md` focused on library behavior and public API usage. Keep
`INSTALL.md` focused on requirements, CMake workflows, installation, and
consumption. Ensure commands are runnable from the stated directory, examples
use current public names, relative links resolve, and version claims match
`config.hpp`.

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
