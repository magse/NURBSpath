#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point2.hpp"

#include <concepts>
#include <limits>
#include <vector>

namespace nurbspath {

/**
 * @brief Owning, detached definition data for a 2D NURBS spline.
 *
 * This aggregate intentionally excludes the polynomial degree. It can be
 * edited as a draft or snapshot and may temporarily contain an incomplete or
 * invalid combination of fields. A `nurbs_spline2` constructor or
 * `set_definition` call validates the complete candidate before adopting it.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct spline2_definition {
    /** @brief Control points in the independent 2D world. */
    std::vector<point2<REAL>> control_points;

    /** @brief Rational weight paired with every control point. */
    std::vector<REAL> weights;

    /** @brief Complete nondecreasing knot vector. */
    std::vector<REAL> knots;

    /** @brief Whether active-domain endpoints must form a closed seam. */
    bool closed = false;

    /** @brief Definition and parameter-boundary tolerance. */
    REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon();
};

} // namespace nurbspath
