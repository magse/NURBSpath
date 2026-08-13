#pragma once

#include "nurbspath/config.hpp"

/** @brief Public types and algorithms for the nurbspath header-only library. */
namespace nurbspath {}

// One-stop public include. Individual headers remain usable for faster builds.
#include "nurbspath/circle2.hpp"
#include "nurbspath/creators.hpp"
#include "nurbspath/distance.hpp"
#include "nurbspath/graphics.hpp"
#include "nurbspath/intersections.hpp"
#include "nurbspath/manual.hpp"
#include "nurbspath/nurbs_spline2.hpp"
#include "nurbspath/nurbs_spline3.hpp"
#include "nurbspath/plane3.hpp"
#include "nurbspath/point2.hpp"
#include "nurbspath/point3.hpp"
#include "nurbspath/projection.hpp"
#include "nurbspath/ray2.hpp"
#include "nurbspath/ray3.hpp"
#include "nurbspath/sphere3.hpp"
#include "nurbspath/utility.hpp"
#include "nurbspath/vector2.hpp"
#include "nurbspath/vector3.hpp"
