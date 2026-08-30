# nurbspath

`nurbspath` 0.3.0 is a dependency-free, header-only C++20 geometry library for
two- and three-dimensional paths and tolerance-aware numerical queries. It
provides strongly typed vectors, points, rays, NURBS curves, circles, spheres,
and infinite planes. The 2D and 3D Cartesian worlds are separate; explicit
projection through a `plane3` is their only entity-conversion bridge.

The project is intended as a supporting geometry library for software focused
on the mechatronic design of 3D-printed robotic arms, crawlers, and
unconventional gearboxes. [NURBS](https://en.wikipedia.org/wiki/Non-uniform_rational_B-spline)
geometry is considered a key technique for automating the design of such
machinery.

This project was created with significant support from ChatGPT. That support
was provided through OpenAI Codex, using the GPT-5 model in an agentic
software-development setting.

The public code uses `snake_case`. Every geometry type is a template whose
floating-point parameter is named `REAL`, so the same API works with `float`,
`double`, and `long double`.

See [INSTALL.md](INSTALL.md) for requirements, build and test commands,
installation, and CMake package consumption.
See [DATA.md](DATA.md) for the versioned, tagged 3D text format and its typed
and heterogeneous reading APIs.

## Features

- `vector3<REAL>` with arithmetic, dot and cross products, norms,
  normalization, projections, rejections, reflection, angles, component
  products, interpolation, and approximate comparison.
- `point3<REAL>` as a position type distinct from a direction, with
  arbitrary-axis rotation about the world origin.
- `ray3<REAL>` with native parameter coordinate `s`.
- `nurbs_spline3<REAL>` with rational evaluation, analytic first, second, and
  third derivatives, unit tangents, approximate arc length, and optional
  closure, plus validated post-construction definition editing. Detached
  `spline2_definition` and `spline3_definition` values group every editable
  field, and the exact `nurbs_defined_spline2` and `nurbs_defined_spline3`
  aliases provide definition-centric spellings. The 2D spline provides the
  corresponding curve operations and general definition editing. Both
  dimensions also provide a named scalar view for optimizer-style single-value
  updates, with deterministic ordering for the current definition.
- Global B-spline interpolation through 3D samples at caller-supplied
  arc-length parameter stations.
- `sphere3<REAL>` parameterized by longitude `u` and latitude `v`.
- `plane3<REAL>` with an orthonormal in-plane `(u, v)` frame.
- Parallel `vector2`, `point2`, `ray2`, `circle2`, and `nurbs_spline2` types in
  an independent 2D Cartesian world.
- Explicit `project(plane3, entity2)` overloads for embedding 2D vectors,
  points, rays, circle center/radius data, and spline control points in 3D.
- Numerically sampled and refined ray/spline intersections with spheres and
  planes, including tangent-contact detection.
- Strictly 2D ray/spline intersections with circles, including tangent contact.
- Tolerance-aware point-to-plane, point-to-sphere, and numerical
  point-to-spline distances with closest parameters and points.
- Strictly 2D point-to-circle and point-to-spline distances.
- Self-contained SVG diagnostics with orthographic and perspective cameras,
  projected geometry markers, and configurable curve tessellation.
- Versioned, single-row tagged text persistence for 3D points, vectors,
  spheres, and NURBS splines, including type-safe heterogeneous input.
- No third-party runtime or build dependency.

## Coordinate and parameter conventions

The 3D types use one shared right-handed Cartesian world coordinate system. The
2D types use a different Cartesian world and cannot implicitly convert to 3D
types. The only public entity bridge is `project(plane3, entity2)`, which uses
the plane origin and orthonormal `(u, v)` directions. Numerical queries between
2D entities are evaluated before and independently of any such projection.

| Entity | Coordinates | Evaluation |
|---|---|---|
| 2D/3D ray | `s >= 0` | `origin + s * direction` in its own world |
| 2D/3D NURBS spline | `s_min() <= s <= s_max()` | Rational B-spline evaluation in its own world |
| 2D circle | `u` angle | Counterclockwise from +X in `[0, 2*pi)` |
| Sphere | `u` longitude, `v` latitude | `u` in `[0, 2*pi)`, `v` in `[-pi/2, pi/2]` |
| Infinite plane | signed in-plane `u`, `v` | `origin + u*u_direction + v*v_direction` |

Ray directions are not normalized automatically. Therefore ray `s` is a
distance only when the supplied direction is a unit vector in that entity's
world. Spline `s` spans the active knot domain. Curves produced by `interpolate`
retain the exact supplied arc-length stations.

`rotate(axis, angle, point)` rotates a 3D point about the oriented axis through
the Cartesian world origin. The axis vector need not be normalized, the signed
angle is measured in radians, and positive angles follow the right-hand rule.

## Quick start

Include the complete public API with:

```cpp
#include <nurbspath/nurbspath.hpp>
```

Every geometry entity also has an initialization-matching allocation helper in
`creators.hpp`. These helpers return `std::shared_ptr`, making ownership
explicit and allowing multiple callers to share the entity lifetime:

```cpp
auto point = nurbspath::make_point2<>(1.0, -0.5);
auto direction = nurbspath::make_vector3<>(1.0, 0.0, 0.0);
auto ray = nurbspath::make_ray3<>(
    nurbspath::point3<double>{0.0, 0.0, 0.0},
    *direction);
auto ray_from_points = nurbspath::make_ray3_from_points<>(
    nurbspath::point3<double>{0.0, 0.0, 0.0},
    nurbspath::point3<double>{2.0, 1.0, 0.0});
auto plane = nurbspath::make_plane3<>(
    nurbspath::vector3<double>{0.0, 0.0, 1.0},
    2.0);
auto plane_from_points = nurbspath::make_plane3_from_points<>(
    nurbspath::point3<double>{0.0, 0.0, 0.0},
    nurbspath::point3<double>{2.0, 0.0, 0.0},
    nurbspath::point3<double>{1.0, 1.0, 0.0});
auto plane_from_u_direction = nurbspath::make_plane3_from_u_direction<>(
    nurbspath::point3<double>{0.0, 0.0, 0.0},
    nurbspath::vector3<double>{1.0, 0.0, 0.0},
    nurbspath::point3<double>{1.0, 1.0, 0.0});
```

Use an explicit scalar argument for zero-value factories, for example
`make_point2<double>()`. Factories that directly forward to entity constructors
preserve their validation and exceptions.
`make_ray2_from_points` and `make_ray3_from_points` derive the unnormalized
direction as `through_point - origin`, so the second point is reached at
`s = 1`. They reject point separations within the selected tolerance.
`make_plane3_from_points` takes the parameter origin, a point in the positive-u
direction, and another point on the plane. `make_plane3_from_u_direction`
replaces the second point with a positive-u direction vector. In both forms,
the third point's component perpendicular to the u-axis defines positive v.

The ordinary 2D and 3D spline factories accept a matching detached definition,
three `std::vector` collections, or three `std::valarray` collections for
control points, weights, and knots. Valarray values are copied in index order
into the spline's vector storage:

```cpp
#include <valarray>

const std::valarray<nurbspath::point3<double>> control_points{
    nurbspath::point3<double>{0.0, 0.0, 0.0},
    nurbspath::point3<double>{1.0, 1.0, 0.0}};
const std::valarray<double> weights{1.0, 1.0};
const std::valarray<double> knots{0.0, 0.0, 1.0, 1.0};
auto spline = nurbspath::make_nurbs_spline3<double>(
    control_points, weights, knots, 1);
```

Create a NURBS path through measured 3D positions:

```cpp
#include <nurbspath/nurbspath.hpp>

#include <vector>

using real = double;
using nurbspath::point3;

const std::vector<point3<real>> samples{
    {0.0, 0.0, 0.0},
    {1.0, 0.2, 0.3},
    {2.0, 0.8, 0.6},
    {3.0, 1.8, 0.2},
    {4.0, 3.0, 0.0}
};
const std::vector<real> arc_length_stations{0.0, 1.1, 2.3, 3.6, 5.2};

const auto path = nurbspath::nurbs_spline3<real>::interpolate(
    samples, arc_length_stations, 3);

const auto value = path.derivatives_at(2.0);
const point3<real> position = value.point;
const nurbspath::vector3<real> velocity = value.first;
const nurbspath::vector3<real> acceleration = value.second;
const nurbspath::vector3<real> jerk = path.third_derivative(2.0);
const nurbspath::vector3<real> unit_tangent = path.tangent(2.0);
```

`interpolate` performs global, exact interpolation with averaged clamped knots
and unit weights. The requested degree is reduced to one less than the number
of samples when necessary. Sample and station counts must match; at least two
samples are required; stations must be strictly increasing. `adopt_to_points`
provides the same operation as a mutating member.

## Constructing a NURBS spline directly

For a degree `p` curve with `C = n + 1` control points, provide:

- `C` world-space control points;
- `C` finite positive weights;
- `C + p + 1` nondecreasing knots (equivalently `n + p + 2`);
- degree `p >= 1`, with `p < control_point_count`.

The active parameter domain is `[knots[p], knots[C]]`. The shared helper
calculates the required allocation size before either a 2D or 3D spline is
constructed:

```cpp
constexpr std::size_t degree = 5;
constexpr std::size_t control_point_count = 7;
constexpr std::size_t knot_count =
    nurbspath::nurbs_knot_count(degree, control_point_count); // 13
```

The helper throws `std::invalid_argument` for a zero degree or when the
control-point count does not exceed the degree. It throws
`std::overflow_error` when the required count cannot fit in `std::size_t`.

```cpp
const double root_half = std::sqrt(0.5);
const nurbspath::nurbs_spline3<double> quarter_circle(
    {{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0}},
    {1.0, root_half, 1.0},
    {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
    2);
```

The complete-definition constructor has an overload whose arguments after
`degree` are `closed` and an optional `tolerance`:

```cpp
const nurbspath::nurbs_spline2<double> closed_triangle(
    {{1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {1.0, 0.0}},
    {1.0, 1.0, 1.0, 1.0},
    {0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
    1,
    true); // closed
```

A closed definition must evaluate to the same point at both ends of its
active domain. `interpolate` and `adopt_to_points` provide matching `closed`
overloads. Closed interpolation input has coincident first and final positions
at strictly increasing stations; open input may end elsewhere. Both forms use
averaged clamped knots and preserve every supplied station.

`is_closed()` reports whether the spline was constructed as closed.
`get_start()` and `get_end()` return cached endpoint positions without another
evaluation. For a zero-based domain, `get_start()` is the point at `s = 0`.
Endpoints are derived, read-only values: there are no `set_start` or `set_end`
functions. To target different endpoints, edit the control-point definition
with `set_control_point` or `set_control_points`; for non-clamped knots, an
endpoint can depend on more than the first or last control point.

Position, first derivative, and second derivative are evaluated together by
`derivatives_at(s)`. Convenience methods `evaluate`, `point_at`,
`first_derivative`, `second_derivative`, `third_derivative`, and `tangent` are
also available. If repeated knots create zero-width spans at an active-domain
boundary, evaluation skips them: `s_min()` uses the first nonzero span to its
right and `s_max()` uses the last nonzero span to its left.

The third rational derivative is evaluated analytically from homogeneous basis
derivatives and the quotient rule. It does not require degree three: basis and
weight derivatives above the polynomial degree are zero, but varying rational
weights can still produce nonzero higher derivatives. For unit weights, a
derivative above the polynomial degree is zero. At an internal knot where the
curve is not third-order continuous, `third_derivative(s)` returns the
right-hand span value; at `s_max()` it returns the left-hand span value.

## Editing a spline definition

The live definition inside a spline remains private so every adopted edit can
be validated and cached endpoints can be refreshed. Public
`spline2_definition<REAL>` and `spline3_definition<REAL>` aggregates provide
owning, detached drafts and snapshots. They contain `control_points`,
`weights`, `knots`, `closed`, and `tolerance`; the spline degree is deliberately
separate. A detached value may be incomplete or temporarily invalid while it
is being edited.

Both ordinary spline constructors accept a detached definition followed by its
degree. `nurbs_defined_spline2<REAL>` and `nurbs_defined_spline3<REAL>` are
exact aliases of `nurbs_spline2<REAL>` and `nurbs_spline3<REAL>`, respectively,
rather than distinct runtime types. They therefore retain every ordinary
spline operation and interoperate with projection, numerical queries,
graphics, and tagged I/O without conversion:

```cpp
nurbspath::spline3_definition<double> initial{
    {{1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0}},
    {1.0, std::sqrt(0.5), 1.0},
    {0.0, 0.0, 0.0, 1.0, 1.0, 1.0},
    false,
    1.0e-12};

nurbspath::nurbs_defined_spline3<double> editable(initial, 2);

// The constructor cloned this lvalue. Later edits to it are still detached.
initial.weights[1] = 0.75;

auto candidate = editable.definition();
candidate.control_points[1].z = 0.25;
candidate.weights[1] = 0.75;
for (double& knot : candidate.knots) {
    knot = 10.0 + 4.0 * knot;
}

// Validate and adopt every editable field together; degree remains 2.
editable.set_definition(candidate);
```

`definition()` always returns a deep-copy snapshot. Changing the snapshot or
the original constructor argument cannot change the path implicitly. The path
changes only after `set_definition(snapshot)` validates and adopts the complete
candidate. If validation throws, the spline, its cached endpoints, and its
degree remain unchanged. An rvalue definition can be moved into either the
constructor or setter.

The corresponding allocation helpers also accept detached definitions. Both
names below allocate the same exact spline type with shared ownership:

```cpp
auto ordinary_owner = nurbspath::make_nurbs_spline3(initial, 2);
auto defined_owner = nurbspath::make_nurbs_defined_spline3(initial, 2);
```

Equivalent constructors, snapshots, setters, aliases, and creator overloads
are available for 2D definitions.

As an alternative to editing a detached definition, both spline dimensions
expose selected numeric definition data through `number_of_parameters()`,
`get_parameter(n)`, `parameter_name(n)`, and `set_parameter(n, value)`. These
are definition parameters, not values of the native curve coordinate `s`.
Let `D` be the dimension (`2` or `3`), `C` the control-point count, and `M` the
knot count. Because every valid spline has one weight per control point, the
flattened zero-based order is:

| Flattened index | Name | Value |
|---|---|---|
| `D * i + c` | `P[i].x`, `P[i].y`, or `P[i].z` | Coordinate `c` of control point `i`, with `c` ordered x, y, then z in 3D |
| `D * C + i` | `W[i]` | Weight `i` |
| `(D + 1) * C + j` | `K[j]` | Knot `j` |

Thus `number_of_parameters()` returns `(D + 1) * C + M`. Coordinates are
point-major: a 3D spline begins `P[0].x`, `P[0].y`, `P[0].z`, `P[1].x`, while
a 2D spline omits each z coordinate. All names and bracketed vector indices
are zero-based. Degree, `closed`, and tolerance are deliberately absent and
remain fixed during every `set_parameter` call. Degree remains immutable;
closure and tolerance are changed through `set_closed`, `set_tolerance`, or a
complete definition update. Derived endpoints are also absent. If a bulk
definition change alters `C` or `M`, callers must query the count and names
again because the later flattened indices move.

For the 3D `editable` spline above, which has three control points and six
knots:

```cpp
const std::size_t count = editable.number_of_parameters(); // 18
const auto name = editable.parameter_name(3);              // "P[1].x"
const double old_x = editable.get_parameter(3);            // 1.0

const bool changed = editable.set_parameter(3, 1.25);      // true
const std::size_t middle_weight =
    3 * editable.control_points().size() + 1;
const bool invalid =
    editable.set_parameter(middle_weight, -1.0);            // false
```

`get_parameter` and `parameter_name` throw `std::out_of_range` for an index
outside the current count. Name construction can also propagate
`std::bad_alloc`. `set_parameter` instead returns false for an out-of-range
index or when immediate full-definition validation rejects the candidate,
including non-finite coordinates or knots, nonpositive weights, invalid knot
ordering or active domains, near-zero homogeneous endpoint weights, and broken
closed seams. False leaves the definition, active domain, and cached endpoints
unchanged. A valid value equal to the current value still returns true because
the candidate was successfully validated and committed.
Allocation failures such as `std::bad_alloc` propagate rather than being
reported as validation failure, while preserving the original spline.

Each `set_parameter` call clones the complete definition, changes one scalar,
validates the whole candidate, and refreshes cached endpoints before commit.
This makes repeated scalar editing convenient for generic optimization and
fitting code, but slower than applying many related changes in one detached or
bulk definition update. It also provides no temporarily invalid staging:
closed-seam endpoint changes, knot rescaling, and other correlated edits should
use `set_definition`. A successful scalar update has the same reference,
pointer, and iterator invalidation rules as the other definition setters.
Because `nurbs_defined_spline2` and `nurbs_defined_spline3` are exact aliases,
the scalar members are visible through those spellings too; the ordinary and
definition-centric names describe editing workflows, not distinct runtime
interfaces.

For smaller, type-specific edits, both `nurbs_spline2` and `nurbs_spline3`
provide indexed `control_point`, `weight`, and `knot` getters with matching
`set_control_point`, `set_weight`, and `set_knot` operations. Whole-vector
`set_control_points`, `set_weights`, and `set_knots` operations support
coordinated edits without exposing mutable live storage:

```cpp
nurbspath::nurbs_spline3<double> editable = quarter_circle;

auto control = editable.control_point(1);
control.z = 0.25;
editable.set_control_point(1, control);
editable.set_weight(1, 0.75);

// Endpoints are available through read-only member calls.
const auto start = editable.get_start();
const auto end = editable.get_end();

// Deliberate endpoint placement goes through the control-point definition.
auto endpoint_controls = editable.control_points();
endpoint_controls.front() = {1.0, 0.0, 0.1};
endpoint_controls.back() = {0.0, 1.0, 0.1};
editable.set_control_points(endpoint_controls);

auto rescaled_knots = editable.knots();
for (double& knot : rescaled_knots) {
    knot = 10.0 + 4.0 * knot;
}
editable.set_knots(rescaled_knots);

// Replace the knots with a standard open-clamped vector on [0, 4].
editable.set_standard_knots(4.0);

// The two-argument form selects both ends of the native s domain.
editable.set_standard_knots(-2.0, 6.0);
```

`nurbs_spline3::set_standard_knots(end)` uses zero as the domain start, while
`set_standard_knots(start, end)` accepts finite bounds with a finite positive
separation. For a single-span spline, the resulting vector is exactly one
half start values and one half end values. For a multi-span spline, the first
and last `degree + 1` knots are clamped to the endpoints and the remaining
`control_point_count - degree - 1` simple interior knots are spaced uniformly.
The total returned by `nurbs_knot_count` can therefore be odd. The active
parameter domain becomes `[start, end]` without changing the spline's degree,
control points, weights, closure state, or tolerance. An interval that is too
narrow in `REAL` to represent distinct, numerically usable knot spans is
rejected.

`set_closed` and `set_tolerance` change the remaining mutable definition
fields. The five-argument
`set_definition(control_points, weights, knots, closed, tolerance)` and the
single-argument `set_definition(definition)` overload replace them together
when related counts or a closed seam must change atomically. Every mutation
builds and validates a complete candidate first; invalid input throws without
changing the current spline. Successful changes refresh `get_start()` and
`get_end()` as well as the active parameter domain. These mutation functions
retain the current degree, and there is no degree setter. A successful setter
replaces the stored definition, so references, pointers, and iterators
previously obtained from its getters must not be reused. For a closed spline,
update both seam controls in one bulk or definition-based call so validation
never observes a temporarily broken seam.

## Rays and surfaces

```cpp
const nurbspath::ray3<double> ray(
    {-3.0, 0.0, 0.0},
    {1.0, 0.0, 0.0});

const nurbspath::sphere3<double> sphere(
    {0.0, 0.0, 0.0},
    1.0);

const nurbspath::plane3<double> plane(
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 1.0},
    {1.0, 0.0, 0.0}); // optional u-direction hint

// Classical Hessian normal form: n_hat dot x = signed_distance.
const nurbspath::plane3<double> offset_plane(
    nurbspath::vector3<double>{1.0, 2.0, 3.0},
    4.0);
```

`sphere.point_at(u, v)` and `plane.point_at(u, v)` convert surface parameters
to world positions. `parameters_of` performs the inverse surface mapping.
`plane.project(point)` orthogonally projects a position onto the plane.

The point-plus-normal constructor chooses a stable `u` direction automatically.
Its overload with a `u` hint projects that hint into the plane before
normalizing it.

The `make_plane3_from_points` allocation helper fixes the first point at
`(u, v) = (0, 0)` and normalizes the direction from the first point to the
second as positive u. `make_plane3_from_u_direction` accepts that positive-u
direction directly. For both helpers, the third point need not be perpendicular
to u: its off-axis component selects positive v, and `u` cross `v` gives the
normal of the right-handed frame. The helpers reject a near-zero u direction
and a third point within tolerance of the u-axis.

The normal-plus-distance constructor accepts any finite signed distance and any
nonzero normal direction or magnitude. It normalizes the supplied normal and
uses the Hessian equation `n_hat dot x = d`, so `d` is the signed perpendicular
distance from the world origin. Its `(u, v)` origin is the closest point
`d*n_hat`. Every infinite plane can be represented this way: tangentially
shifting a point does not produce a different plane. Use the point-plus-normal
constructor when a particular point must instead be `(u, v) = (0, 0)`.

For coefficients written as `a*x + b*y + c*z = k`, pass normal `(a, b, c)` and
signed distance `k / sqrt(a*a + b*b + c*c)`. For the alternative convention
`a*x + b*y + c*z + q = 0`, use signed distance
`-q / sqrt(a*a + b*b + c*c)`. `signed_distance_from_origin()` recovers `d`
from a plane constructed by either form.

## Independent 2D geometry and plane projection

The 2D API mirrors the applicable 3D path API without introducing `plane2`:

```cpp
const nurbspath::point2<double> point_2d{1.0, -0.5};
const nurbspath::ray2<double> ray_2d(point_2d, {1.0, 0.25});
const nurbspath::circle2<double> circle_2d({0.0, 0.0}, 2.0);
const auto spline_2d = nurbspath::nurbs_spline2<double>::interpolate(
    {{-1.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}},
    {4.0, 5.5, 7.0},
    2);
```

The runnable `nurbspath_2d_example` demonstrates native points, vectors, a
ray, a circle, an interpolated spline, and 2D intersection and distance
queries without using a projection plane.

These objects live entirely in their own 2D world. Constructors and implicit
conversions do not mix `point2` with `point3`, `vector2` with `vector3`, or
`ray2` with `ray3`. Select a 3D plane explicitly when an embedded object is
required:

```cpp
const nurbspath::plane3<double> drawing_plane(
    {3.0, -2.0, 1.0},  // projected 2D origin
    {0.0, 1.0, 1.0},   // plane normal
    {1.0, 0.0, 0.0});  // positive 2D X / plane-u hint

const nurbspath::point3<double> point_3d =
    nurbspath::project(drawing_plane, point_2d);
const nurbspath::ray3<double> ray_3d =
    nurbspath::project(drawing_plane, ray_2d);
const nurbspath::sphere3<double> sphere_3d =
    nurbspath::project(drawing_plane, circle_2d);
const nurbspath::nurbs_spline3<double> spline_3d =
    nurbspath::project(drawing_plane, spline_2d);
```

For points, 2D `(x, y)` maps to plane `(u, v)`. Vectors map through the plane's
orthonormal basis, so lengths and ray `s` values are preserved. A circle maps
to a complete `sphere3` using only the projected center and unchanged radius;
it does not become a planar 3D circle. A spline maps only its control points;
weights, knots, degree, tolerance, and native `s` domain remain unchanged.
Closure is preserved as well.

The available 2D numerical queries never project their operands:

```cpp
const auto ray_contacts = nurbspath::intersect_ray_circle(
    ray_2d, circle_2d, settings);
const auto spline_contacts = nurbspath::intersect_spline_circle(
    spline_2d, circle_2d, settings);
const auto circle_distance = nurbspath::distance_to_circle(
    point_2d, circle_2d, 1e-9);
const auto spline_distance_2d = nurbspath::distance_to_spline(
    point_2d, spline_2d, settings);
```

An `intersection2` contains a 2D `point`, ray/spline `s`, circle angle `u`, and
the refined 2D residual. Detailed 2D distance results likewise contain a
`point2` closest point and either `u` or `s`.

## SVG diagnostics

`nurbspath/graphics.hpp` provides a lightweight SVG scene renderer with no
third-party dependency. Both projections are defined by an `eyepoint` and a
`lookatpoint` in the shared world coordinate system:

```cpp
const point3<double> eye{5.0, -7.0, 4.5};
const point3<double> look_at{0.0, 0.0, 0.5};

const auto orthographic_view = nurbspath::svg_view3<double>::orthographic(
    eye,
    look_at,
    6.0,  // minimum visible world width
    4.5,  // minimum visible world height
    960,  // SVG pixel width
    720); // SVG pixel height

const auto perspective_view = nurbspath::svg_view3<double>::perspective(
    eye,
    look_at,
    std::numbers::pi_v<double> / 4.0, // vertical view angle in radians
    960,
    720,
    0.05); // near distance in world units
```

The camera derives a stable vertical direction automatically: it prefers world
`+z`, falling back to world `+y` when viewing nearly parallel to `+z`. There is
therefore no third camera parameter or implicit transformation attached to an
entity. Orthographic projection uses one uniform scale so circles remain
circles; if viewport and image aspect ratios differ, the looser axis is
letterboxed and both requested world extents remain visible. Perspective
projection preserves the image aspect ratio and clips geometry at the near
plane.

Add several entities to one document and emit the resulting XML:

```cpp
nurbspath::svg_graphics_options3<double> graphics;
graphics.line_width = 1.5;          // SVG/viewBox units (normally pixels)
graphics.spline_segment_count = 160;
graphics.sphere_segment_count = 120;
graphics.plane_square_side_length = 0.25; // world units
graphics.plane_parameter_axis_length = 0.5;

nurbspath::svg_document3<double> document(perspective_view, graphics);
document.add(point);
document.add(ray);
document.add(sphere);
document.add(plane);
document.add(path);
document.add(drawing_plane, point_2d);
document.add(drawing_plane, ray_2d);
document.add(drawing_plane, circle_2d);
document.add(drawing_plane, spline_2d);

std::ofstream output("scene.svg");
document.write(output);
```

For a flat SVG of the native 2D world, use `svg_canvas2`. This path has no
camera, `plane3`, projection, perspective, or depth. The view is defined by a
2D center and minimum visible width/height:

```cpp
const nurbspath::svg_view2<double> flat_view(
    {0.0, 0.0}, // 2D world point at image center
    6.0,        // minimum visible 2D width
    4.5,        // minimum visible 2D height
    960,
    720);

nurbspath::svg_graphics_options2<double> flat_graphics;
flat_graphics.line_width = 1.5;
flat_graphics.spline_segment_count = 160;

nurbspath::svg_canvas2<double> canvas(flat_view, flat_graphics);
canvas.add(point_2d);
canvas.add(ray_2d);
canvas.add(circle_2d);
canvas.add(spline_2d);

std::ofstream flat_output("scene_2d.svg");
canvas.write(flat_output);
```

Only `point2`, `ray2`, `circle2`, and `nurbs_spline2` have flat-canvas `add`
overloads. Free `vector2` objects and every 3D type are rejected at compile
time. All canvas lines—including the ray support, positive direction, circle,
spline, and control polygon—are solid. The established black, pale-red,
pale-blue, and violet colors and location dots are retained. A unit X/Y
coordinate reference is always drawn at the 2D origin with red/green half-width
axes and labels. The mapping uses a uniform scale, so circles remain circles;
extra space is letterboxed when aspect ratios differ.

`document.svg()` and `canvas.svg()` return complete SVG strings. For one entity,
`to_svg(view, entity, options)` selects the flat or 3D overload from the view
type. Rendering follows `add` order and the background is transparent. Every
document automatically starts with a positive unit-length world coordinate
system at `(0, 0, 0)`: X is
red, Y is green, and Z is blue. These axes use half the configured line width
and have matching `X`, `Y`, and `Z` labels at their tips.

The diagnostic marks are deliberately simple and predictable:

- `point3` is a black dot with radius equal to `line_width`, hence diameter
  twice the line width;
- `ray3` has a pale-red dashed supporting infinite line extended to the SVG
  edges, a stronger solid one-world-unit line from its origin along the
  normalized positive parameter direction, and a stronger red origin dot;
- `sphere3` has a pale blue apparent silhouette and its `v=0` world-xy equator;
  self-hidden equator elements use a still paler blue, and a stronger blue dot
  marks the center;
- `plane3` is a pale green square centered on the plane origin with default side
  length `0.25`, plus a unit-length normal, half-unit lines along positive `u`
  and positive `v`, and a stronger green origin dot;
- `nurbs_spline3` is approximated by configurable, uniformly spaced black
  straight elements over its full active `s` domain; muted violet control-point
  dots are connected by a faint dashed control polygon, and larger black dots
  mark the evaluated start and end points;
- `point2`, `ray2`, `circle2`, and `nurbs_spline2` are added only as
  `document.add(plane, entity)` and retain the colors of their projected 3D
  forms; primary 2D entity strokes are dash-dotted, while auxiliary, hidden,
  positive-direction, and control-polygon strokes are dotted;
- `vector2` and `vector3` have no rendering overload because an unpositioned
  vector has no world-space location.

SVG output clips to the image viewport. Perspective output also clips segments
against the near plane. A line passing exactly through the eyepoint can collapse
to a projected point and is omitted. Tessellated curves and sphere marks become
smoother as their segment counts increase.

The runnable `nurbspath_svg_example` writes matching orthographic and
perspective scenes plus a native flat `nurbspath_2d.svg` scene. See
[INSTALL.md](INSTALL.md) for its command.

## Numerical intersections

The explicitly named functions are:

```cpp
intersect_ray_sphere(ray, sphere, settings);
intersect_ray_plane(ray, plane, settings);
intersect_spline_sphere(spline, sphere, settings);
intersect_spline_plane(spline, plane, settings);
intersect_ray_circle(ray_2d, circle_2d, settings);
intersect_spline_circle(spline_2d, circle_2d, settings);
```

Equivalent overloaded `intersect` functions are available. Results have an
`intersection_kind`:

- `none`: no forward-ray or in-domain spline contact;
- `discrete`: `points` contains finite contacts sorted by increasing `s`;
- `coincident`: the ray or sampled spline lies continuously on the plane or
  sphere, so `points` is empty because a finite list cannot represent the set.

Each `intersection3` contains its world `point`, ray/spline `s`, surface `u`
and `v`, and the absolute surface `residual` at the refined contact.
Each `intersection2` instead contains a `point2`, ray/spline `s`, circle angle
`u`, and a residual evaluated entirely in the 2D world. For a closed spline,
a contact shared by `s_min()` and `s_max()` is reported once at `s_min()`.

Configure numerical behavior through `numerical_settings<REAL>`:

```cpp
nurbspath::numerical_settings<double> settings;
settings.residual_tolerance = 1e-9;   // surface-distance units
settings.parameter_tolerance = 1e-11; // s units
settings.max_iterations = 100;
settings.sample_count = 512;

const auto contacts = nurbspath::intersect(path, plane, settings);
```

Intersection searches sample a finite parameter interval, refine sign changes
with bisection, and minimize local absolute residuals to retain tangent contacts.
Ray/sphere searches derive a safe forward bound from the sphere location and
radius. Ray/plane searches classify parallel and coplanar rays before numerical
refinement. Spline searches use the complete active knot domain.

`sample_count` is the resolution contract: it must be high enough to expose the
smallest oscillation or contact basin in a curve. Increase it for high-degree,
highly oscillatory, or large-domain splines. A numerical method cannot guarantee
finding an arbitrarily narrow unsampled feature.

## Tolerance-aware distances

Detailed functions return the closest point and associated parameters:

```cpp
const auto plane_result = nurbspath::distance_to_plane(point, plane, 1e-9);
const auto sphere_result = nurbspath::distance_to_sphere(point, sphere, 1e-9);
const auto spline_result = nurbspath::distance_to_spline(point, path, settings);
const auto circle_result_2d = nurbspath::distance_to_circle(
    point_2d, circle_2d, 1e-9);
const auto spline_result_2d = nurbspath::distance_to_spline(
    point_2d, spline_2d, settings);

double closest_s = spline_result.s;
point3<real> closest_point = spline_result.closest_point;
```

Scalar overloads named `distance` return only the distance. A measured distance
less than or equal to the selected residual tolerance is returned as zero.
Plane distance uses orthogonal projection and sphere distance uses radial
projection. Spline distance samples the full active domain, identifies every
sampled local basin, and refines each with golden-section minimization; its
global guarantee is therefore relative to `sample_count`.
The circle and 2D spline overloads perform the analogous calculations using
only `point2` and `vector2` arithmetic. A projection plane is neither accepted
nor consulted by any 2D distance or intersection function. Point-to-spline
distance and total arc-length approximation both use the spline's active knot
domain.

## Public API summary

`vector2<REAL>` is an aggregate with public `x` and `y` components that default
to zero. It mirrors the applicable vector operations in two dimensions and adds
a zero-argument `normalize()` operation for in-place unit normalization, a
signed scalar `cross`, left/right perpendicular vectors, and `signed_angle_to`.
`point2<REAL>` is likewise an aggregate with public,
zero-defaulted `x` and `y` coordinates. It preserves the point-versus-vector
type boundary and provides `magnitude()` as Euclidean distance and
`manhattan_distance()` as L1 distance from the 2D origin.
`ray2<REAL>` uses forward parameter `s`, and `circle2<REAL>` provides
`point_at(u)`, `normal_at`, and `parameter_of`.

`point2`, `vector2`, `point3`, and `vector3` support stream insertion and
extraction with `operator<<` and `operator>>`. Their text format is
whitespace-separated Cartesian coordinates: `x y` in 2D and `x y z` in 3D.
Extraction changes a value only after all of its coordinates are read
successfully.

The same four types provide `csv_write(output, format, decimal_places)` and
`csv_read(input, format)`, where `format` is `text_format::csv`,
`text_format::tsv`, or `text_format::txt`. CSV is the default. `csv_write`
always uses scientific notation; `decimal_places` selects the number of digits
after the decimal point and defaults to six.
The methods write or read one coordinate record without adding a line ending.
For example:

```cpp
nurbspath::point3<double> point;
point.csv_read(input, nurbspath::text_format::tsv);
point.csv_write(output, nurbspath::text_format::csv, 8);
// 0.00000000e+00,0.00000000e+00,0.00000000e+00
```

Their `write(output)` and `read(input)` members provide compact binary I/O.
The binary record contains two or three native `REAL` values in X/Y[/Z] order,
without a header. It is intended for matching readers and writers and is not
portable between different scalar types, floating-point representations, or
byte orders. Text and binary reads leave the existing object unchanged unless
the complete record is available.

### Tagged 3D persistence

`point3`, `vector3`, `sphere3`, and `nurbs_spline3` each provide
`tag_write(tag, output)` and a static `tag_read(input)`. Every call writes one
versioned text row whose floating-point fields use round-trip scientific
notation:

```text
42 point3 v1 1.0000000000000000e+00 2.0000000000000000e+00 3.0000000000000000e+00
42 sphere3 v1 0.0000000000000000e+00 0.0000000000000000e+00 0.0000000000000000e+00 2.5000000000000000e+00
```

Tags are application data rather than unique keys. Multiple entity rows may
therefore use the same tag, for example to record every entity at one time
step. A typed read returns `std::optional<tagged_read_result<ENTITY>>`, which
contains the tag and a shared pointer to the allocated entity:

```cpp
nurbspath::point3<double>{1.0, 2.0, 3.0}.tag_write(42, output);
auto point_record = nurbspath::point3<double>::tag_read(input);
```

The general `nurbspath::tag_read<REAL>(input)` routine returns
`std::optional<tagged_entity3<REAL>>`. Its `entity` member is a type-safe
variant of shared pointers to the four supported concrete types. The on-disk
token for `nurbs_spline3` is the shorter stable name `spline3`.
`nurbs_defined_spline3` is an exact alias, so it uses that same token and
reader result rather than introducing another stored entity type. Unknown,
malformed, invalid, or surplus fields set `failbit`, and one complete physical
row is consumed per call. See [DATA.md](DATA.md) for the exact `v1` grammar,
spline field order, failure behavior, and portability rules.

`nurbs_spline2<REAL>` exposes the same rational definition, native domain,
analytic derivatives, tangent, arc-length approximation, global
`interpolate`, and `adopt_to_points` operations as the 3D spline, with all
positions and derivatives remaining two-dimensional. The `project` overloads
are the explicit bridge from 2D entities to a selected `plane3` embedding and
preserve spline closure. `spline2_definition`, `definition()`, and both
`set_definition` forms follow the same detached and transactional rules as
their 3D counterparts. Its scalar definition view is exposed through
`number_of_parameters`, `get_parameter`, `parameter_name`, and
`set_parameter`, using point-major x/y coordinates followed by weights and
knots; degree, closure, and tolerance are excluded.

`vector3<REAL>` is an aggregate with public `x`, `y`, and `z` components that
default to zero. It also provides indexed access, exact equality, unary signs,
vector addition/subtraction, scalar multiplication/division, `dot`, `cross`,
`length`, `length_squared`, `normalize`, `normalized`, `is_near_zero`,
`projected_onto`,
`rejected_from`, `reflected`, `angle_to`, `component_product`, `min_component`,
`max_component`, and `approximately_equal`. Free functions provide `dot`,
`cross`, and `lerp`, and static factory functions provide `zero` and the three
Cartesian unit vectors. `normalize()` modifies the vector in place using the
default tolerance and returns no value, while `normalized(tolerance)` returns a
normalized copy.

`point3<REAL>` is an aggregate with public `x`, `y`, and `z` coordinates that
default to zero. It provides indexed access, exact equality, translation by a
vector, subtraction of two points to form a vector, `magnitude()` as Euclidean
distance and `manhattan_distance()` as L1 distance from the world origin,
`approximately_equal`, `origin`, and free `distance`, `distance_squared`, and
`lerp` operations. The free `rotate(axis, angle, point)` operation rotates a
point about a world-origin axis using a signed right-handed angle in radians;
the axis direction is normalized internally.

`ray3<REAL>` provides `origin`, `direction`, `point_at(s)`, `evaluate(s)`, and a
unit `tangent`. `sphere3<REAL>` provides `center`, `radius`, `point_at(u, v)`,
`normal_at`, and `parameters_of`. `plane3<REAL>` provides `origin`, `normal`,
the two basis directions, `point_at(u, v)`, `signed_distance_to`, `project`, and
`parameters_of`, plus `signed_distance_from_origin` for its equivalent Hessian
normal form.

`nurbs_spline3<REAL>` exposes its fields with `control_points`, `weights`,
`knots`, `degree`, and `tolerance`, and clones every editable field except
degree through `definition()`. `spline3_definition<REAL>` is the corresponding
detached aggregate, and `nurbs_defined_spline3<REAL>` is an exact alias of the
ordinary spline type. The spline exposes its domain with `s_min` and `s_max`
and its geometry with `evaluate`, `point_at`, `derivatives_at`,
`first_derivative`, `second_derivative`, `third_derivative`, `tangent`, and
`approximate_arc_length`. Static `interpolate` constructs a new curve, while
`adopt_to_points` replaces an existing one. Both operations accept a `closed`
overload. `is_closed()`, `get_start()`, and `get_end()` expose closure and
cached, read-only endpoint values. Endpoints have no direct setters and are
targeted by editing the control-point definition. Indexed and bulk checked
setters change every definition field except degree, and `set_definition`
performs coordinated whole-definition changes while retaining that degree.
The named scalar view provided by `number_of_parameters`, `get_parameter`,
`parameter_name`, and `set_parameter` flattens point-major x/y/z coordinates,
weights, and knots for validated single-value updates; it excludes degree,
closure, and tolerance.
`set_standard_knots(end)` and `set_standard_knots(start, end)` replace the knot
vector with a validated open-clamped uniform definition on the requested native
parameter domain.

`svg_view3<REAL>` creates validated two-point orthographic or perspective
cameras and reports its mode as `svg_projection3`. The
`svg_graphics_options3<REAL>` type controls line width, spline and sphere
segment counts, the plane square side length, and its positive parameter-axis
marker length. `svg_document3<REAL>` provides entity-specific `add` overloads
plus plane-and-2D-entity overloads, `svg`, and `write`; the free `to_svg`
function renders one supported 3D entity or one plane-embedded 2D entity.
`svg_view2<REAL>`, `svg_graphics_options2<REAL>`, and `svg_canvas2<REAL>`
provide the separate flat, solid-line renderer that accepts native 2D entities
only.

`creators.hpp` provides `make_vector2`, `make_point2`, `make_ray2`,
`make_ray2_from_points`, `make_circle2`, `make_nurbs_spline2`,
`make_nurbs_defined_spline2`, `make_vector3`, `make_point3`, `make_ray3`,
`make_ray3_from_points`, `make_sphere3`, all three `make_plane3` constructor
forms, `make_plane3_from_points`, `make_plane3_from_u_direction`,
`make_nurbs_spline3`, and `make_nurbs_defined_spline3`. Every function returns a
shared-ownership smart pointer. Copying a result keeps the same entity alive
until the final owner releases it. The ordinary spline factories provide
definition-value overloads as well as `std::vector` and `std::valarray`
collection overloads, including forms that accept `closed` after `degree`.
The definition-centric factory names accept the matching detached definition
and degree.

Reusable routines in `utility.hpp` include `nurbs_knot_count`,
`approximately_equal`, `square`, scaled-pivot `solve_linear_system`,
`bisect_root`, `golden_section_minimize`, and sampled `find_roots`. These
routines validate settings and throw standard exceptions for invalid domains,
singular systems, or non-finite residuals. Geometry constructors similarly
reject zero directions/normals, nonpositive radii/weights, invalid knot
definitions, and parameters outside a spline's active domain.

## Version metadata

Every public C++ header, example, and test includes
`nurbspath/config.hpp` directly. The only exception is `config.hpp` itself.

The configuration macros can be inspected without constructing a geometry
object:

```cpp
#include <nurbspath/config.hpp>

#include <iostream>

int main() {
    std::cout << NURBSPATH_GIT_VERSION << '\n';
}
```

`NURBSPATH_VERSION_MAJOR`, `NURBSPATH_VERSION_MINOR`,
`NURBSPATH_VERSION_PATCH`, `NURBSPATH_VERSION_STRING`, and
`NURBSPATH_VERSION_NUMBER` describe the semantic library version.
`NURBSPATH_GIT_COMMIT`, `NURBSPATH_GIT_COMMIT_SHORT`,
`NURBSPATH_GIT_DESCRIBE`, `NURBSPATH_GIT_DIRTY`,
`NURBSPATH_GIT_COMMIT_AVAILABLE`, and `NURBSPATH_GIT_VERSION` describe the
repository state observed by CMake. The checked-in release fallback reports
`0.3.0+v0.3.0`; its commit hash is `unavailable` because a file cannot embed
the hash of the commit that contains itself.

CMake refreshes those Git values during configuration and places its generated
header before the source include directory. Reconfigure after a commit, tag,
or worktree-state change to refresh them. The generated header sets
`NURBSPATH_CONFIG_GENERATED` to one and is the copy installed with the package.
Direct, no-CMake use of the source `include` directory selects the local
fallback, where that macro is zero.

## Header layout

| Header | Contents |
|---|---|
| `nurbspath/config.hpp` | Library semantic version and current Git metadata |
| `nurbspath/vector2.hpp` | 2D vector and free vector operations |
| `nurbspath/point2.hpp` | 2D position and point distance |
| `nurbspath/ray2.hpp` | Forward 2D parametric ray |
| `nurbspath/circle2.hpp` | Parameterized 2D circle |
| `nurbspath/spline2_definition.hpp` | Detached editable 2D spline definition |
| `nurbspath/nurbs_spline2.hpp` | 2D rational spline, derivatives, interpolation |
| `nurbspath/vector3.hpp` | 3D vector and free vector operations |
| `nurbspath/point3.hpp` | 3D position and point distance |
| `nurbspath/ray3.hpp` | Forward parametric ray |
| `nurbspath/spline3_definition.hpp` | Detached editable 3D spline definition |
| `nurbspath/nurbs_spline3.hpp` | Rational spline, derivatives, interpolation |
| `nurbspath/sphere3.hpp` | Parameterized sphere |
| `nurbspath/serialization.hpp` | Coordinate and tagged-record I/O support |
| `nurbspath/tagged_serialization.hpp` | Type-safe general tagged 3D reader |
| `nurbspath/plane3.hpp` | Parameterized infinite plane |
| `nurbspath/projection.hpp` | Explicit plane-based 2D-to-3D entity projection |
| `nurbspath/creators.hpp` | `std::shared_ptr` factories for all geometry entities |
| `nurbspath/graphics.hpp` | Flat 2D and 3D SVG diagnostics |
| `nurbspath/manual.hpp` | Extension point under `nurbspath::manual` |
| `nurbspath/utility.hpp` | Knot sizing, settings, root/minimum finders, linear solver |
| `nurbspath/intersections.hpp` | 2D circle and 3D surface intersections |
| `nurbspath/distance.hpp` | 2D and 3D point-entity distances |
| `nurbspath/nurbspath.hpp` | Complete public API |

`examples/basic_usage.cpp` and `examples/basic_usage_2d.cpp` are runnable 3D
and native 2D geometry examples. `examples/svg_usage.cpp` writes orthographic
and perspective 3D scenes plus a native 2D scene. Numbered files under `tests/`
provide focused, dependency-free CTest programs. Doxygen HTML can be built
through the optional `docs` target described in [INSTALL.md](INSTALL.md).
