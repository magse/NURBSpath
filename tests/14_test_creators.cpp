#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <memory>
#include <stdexcept>
#include <type_traits>
#include <valarray>

int main() {
    using namespace test_support;

    auto vector_2d = nurbspath::make_vector2(3.0, 4.0);
    auto point_2d = nurbspath::make_point2(1.0, -2.0);
    auto ray_2d = nurbspath::make_ray2(*point_2d, *vector_2d);
    const point2<real> through_point_2d{4.0, 2.0};
    auto ray_2d_from_points =
        nurbspath::make_ray2_from_points(*point_2d, through_point_2d);
    auto circle_2d = nurbspath::make_circle2(*point_2d, 2.5);
    auto spline_2d = nurbspath::make_nurbs_spline2<real>(
        {{0.0, 0.0}, {2.0, 1.0}},
        {1.0, 1.0},
        {4.0, 4.0, 8.0, 8.0},
        1);
    auto closed_spline_2d = nurbspath::make_nurbs_spline2<real>(
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
        1,
        true);
    const std::valarray<point2<real>> valarray_control_points_2d{
        point2<real>{0.0, 0.0}, point2<real>{2.0, 1.0}};
    const std::valarray<real> valarray_weights_2d{1.0, 1.0};
    const std::valarray<real> valarray_knots_2d{4.0, 4.0, 8.0, 8.0};
    auto valarray_spline_2d = nurbspath::make_nurbs_spline2(
        valarray_control_points_2d,
        valarray_weights_2d,
        valarray_knots_2d,
        1);
    const std::valarray<point2<real>> closed_valarray_controls_2d{
        point2<real>{0.0, 0.0},
        point2<real>{1.0, 0.0},
        point2<real>{0.0, 1.0},
        point2<real>{0.0, 0.0}};
    const std::valarray<real> closed_valarray_weights_2d{
        1.0, 1.0, 1.0, 1.0};
    const std::valarray<real> closed_valarray_knots_2d{
        0.0, 0.0, 1.0, 2.0, 3.0, 3.0};
    auto closed_valarray_spline_2d = nurbspath::make_nurbs_spline2(
        closed_valarray_controls_2d,
        closed_valarray_weights_2d,
        closed_valarray_knots_2d,
        1,
        true);

    static_assert(std::is_same_v<
        decltype(vector_2d), std::unique_ptr<vector2<real>>>);
    static_assert(std::is_same_v<
        decltype(point_2d), std::unique_ptr<point2<real>>>);
    static_assert(std::is_same_v<
        decltype(ray_2d), std::unique_ptr<ray2<real>>>);
    static_assert(std::is_same_v<
        decltype(ray_2d_from_points), std::unique_ptr<ray2<real>>>);
    static_assert(std::is_same_v<
        decltype(circle_2d), std::unique_ptr<circle2<real>>>);
    static_assert(std::is_same_v<
        decltype(spline_2d), std::unique_ptr<nurbs_spline2<real>>>);
    static_assert(std::is_same_v<
        decltype(valarray_spline_2d), std::unique_ptr<nurbs_spline2<real>>>);
    check(vector_2d->x == 3.0 && vector_2d->y == 4.0,
          "make_vector2 public components");
    check_near(vector_2d->length(), 5.0, 1e-12, "make_vector2 value");
    check(point_2d->x == 1.0 && point_2d->y == -2.0,
          "make_point2 public coordinates");
    check_point2(*point_2d, {1.0, -2.0}, 1e-12, "make_point2 value");
    check_point2(ray_2d->evaluate(2.0), {7.0, 6.0}, 1e-12,
                 "make_ray2 value");
    check(ray_2d->direction() == *vector_2d,
          "make_ray2 preserves the supplied direction");
    check(ray_2d_from_points->direction() == *vector_2d,
          "make_ray2_from_points keeps the point separation unnormalized");
    check_point2(ray_2d_from_points->evaluate(1.0), through_point_2d, 1e-12,
                 "make_ray2_from_points reaches the second point at s=1");
    check_near(circle_2d->radius(), 2.5, 0.0, "make_circle2 value");
    check_point2(spline_2d->evaluate(6.0), {1.0, 0.5}, 1e-12,
                 "make_nurbs_spline2 value");
    check_point2(valarray_spline_2d->evaluate(6.0), {1.0, 0.5}, 1e-12,
                 "make_nurbs_spline2 valarray value");
    check(closed_spline_2d->is_closed(),
          "make_nurbs_spline2 forwards closure");
    check(closed_valarray_spline_2d->is_closed(),
          "2D valarray factory forwards closure");

    auto vector_3d = nurbspath::make_vector3(1.0, 2.0, 2.0);
    auto point_3d = nurbspath::make_point3(2.0, -1.0, 3.0);
    auto ray_3d = nurbspath::make_ray3(*point_3d, *vector_3d);
    const point3<real> through_point_3d{3.0, 1.0, 5.0};
    auto ray_3d_from_points =
        nurbspath::make_ray3_from_points(*point_3d, through_point_3d);
    auto sphere_3d = nurbspath::make_sphere3(*point_3d, 1.5);
    auto hessian_plane = nurbspath::make_plane3(
        vector3<real>{0.0, 0.0, 2.0}, 3.0);
    auto origin_plane = nurbspath::make_plane3(
        point3<real>{1.0, 2.0, 3.0}, vector3<real>{0.0, 1.0, 0.0});
    auto oriented_plane = nurbspath::make_plane3(
        point3<real>{1.0, 2.0, 3.0},
        vector3<real>{0.0, 0.0, 1.0},
        vector3<real>{1.0, 0.0, 0.0});
    const point3<real> frame_origin{1.0, 2.0, 3.0};
    const point3<real> frame_plane_point{4.0, 5.0, 3.0};
    auto three_point_plane = nurbspath::make_plane3_from_points(
        frame_origin,
        point3<real>{3.0, 2.0, 3.0},
        frame_plane_point);
    auto u_direction_plane = nurbspath::make_plane3_from_u_direction(
        frame_origin,
        vector3<real>{0.0, 0.0, 4.0},
        point3<real>{1.0, 5.0, 5.0});
    auto spline_3d = nurbspath::make_nurbs_spline3<real>(
        {{0.0, 0.0, 0.0}, {2.0, 4.0, 6.0}},
        {1.0, 1.0},
        {2.0, 2.0, 6.0, 6.0},
        1);
    auto closed_spline_3d = nurbspath::make_nurbs_spline3<real>(
        {{0.0, 0.0, 0.0},
         {1.0, 0.0, 0.0},
         {0.0, 1.0, 0.0},
         {0.0, 0.0, 0.0}},
        {1.0, 1.0, 1.0, 1.0},
        {0.0, 0.0, 1.0, 2.0, 3.0, 3.0},
        1,
        true);
    const std::valarray<point3<real>> valarray_control_points_3d{
        point3<real>{0.0, 0.0, 0.0}, point3<real>{2.0, 4.0, 6.0}};
    const std::valarray<real> valarray_weights_3d{1.0, 1.0};
    const std::valarray<real> valarray_knots_3d{2.0, 2.0, 6.0, 6.0};
    auto valarray_spline_3d = nurbspath::make_nurbs_spline3(
        valarray_control_points_3d,
        valarray_weights_3d,
        valarray_knots_3d,
        1);
    const std::valarray<point3<real>> closed_valarray_controls_3d{
        point3<real>{0.0, 0.0, 0.0},
        point3<real>{1.0, 0.0, 0.0},
        point3<real>{0.0, 1.0, 0.0},
        point3<real>{0.0, 0.0, 0.0}};
    const std::valarray<real> closed_valarray_weights_3d{
        1.0, 1.0, 1.0, 1.0};
    const std::valarray<real> closed_valarray_knots_3d{
        0.0, 0.0, 1.0, 2.0, 3.0, 3.0};
    auto closed_valarray_spline_3d = nurbspath::make_nurbs_spline3(
        closed_valarray_controls_3d,
        closed_valarray_weights_3d,
        closed_valarray_knots_3d,
        1,
        true);

    static_assert(std::is_same_v<
        decltype(vector_3d), std::unique_ptr<vector3<real>>>);
    static_assert(std::is_same_v<
        decltype(point_3d), std::unique_ptr<point3<real>>>);
    static_assert(std::is_same_v<
        decltype(ray_3d), std::unique_ptr<ray3<real>>>);
    static_assert(std::is_same_v<
        decltype(ray_3d_from_points), std::unique_ptr<ray3<real>>>);
    static_assert(std::is_same_v<
        decltype(sphere_3d), std::unique_ptr<sphere3<real>>>);
    static_assert(std::is_same_v<
        decltype(hessian_plane), std::unique_ptr<plane3<real>>>);
    static_assert(std::is_same_v<
        decltype(three_point_plane), std::unique_ptr<plane3<real>>>);
    static_assert(std::is_same_v<
        decltype(u_direction_plane), std::unique_ptr<plane3<real>>>);
    static_assert(std::is_same_v<
        decltype(spline_3d), std::unique_ptr<nurbs_spline3<real>>>);
    static_assert(std::is_same_v<
        decltype(valarray_spline_3d), std::unique_ptr<nurbs_spline3<real>>>);
    check(vector_3d->x == 1.0 && vector_3d->y == 2.0 && vector_3d->z == 2.0,
          "make_vector3 public components");
    check_near(vector_3d->length(), 3.0, 1e-12, "make_vector3 value");
    check(point_3d->x == 2.0 && point_3d->y == -1.0 && point_3d->z == 3.0,
          "make_point3 public coordinates");
    check_point(*point_3d, {2.0, -1.0, 3.0}, 1e-12, "make_point3 value");
    check_point(ray_3d->evaluate(2.0), {4.0, 3.0, 7.0}, 1e-12,
                "make_ray3 value");
    check(ray_3d->direction() == *vector_3d,
          "make_ray3 preserves the supplied direction");
    check(ray_3d_from_points->direction() == *vector_3d,
          "make_ray3_from_points keeps the point separation unnormalized");
    check_point(ray_3d_from_points->evaluate(1.0), through_point_3d, 1e-12,
                "make_ray3_from_points reaches the second point at s=1");
    check_near(sphere_3d->radius(), 1.5, 0.0, "make_sphere3 value");
    check_near(hessian_plane->signed_distance_from_origin(), 3.0, 1e-12,
               "make_plane3 Hessian overload");
    check_point(origin_plane->origin(), {1.0, 2.0, 3.0}, 1e-12,
                "make_plane3 origin overload");
    check(oriented_plane->u_direction().approximately_equal({1.0, 0.0, 0.0}),
          "make_plane3 u-hint overload");
    check_point(three_point_plane->origin(), frame_origin, 1e-12,
                "three-point plane preserves its first point as origin");
    check(three_point_plane->u_direction().approximately_equal(
              {1.0, 0.0, 0.0}, 1e-12),
          "three-point plane derives positive u from its second point");
    check(three_point_plane->v_direction().approximately_equal(
              {0.0, 1.0, 0.0}, 1e-12),
          "three-point plane orients positive v toward its third point");
    check(three_point_plane->normal().approximately_equal(
              {0.0, 0.0, 1.0}, 1e-12),
          "three-point plane produces a right-handed frame");
    const auto [three_point_u, three_point_v] =
        three_point_plane->parameters_of(frame_plane_point);
    check_near(three_point_u, 3.0, 1e-12,
               "three-point plane retains the third point's u component");
    check_near(three_point_v, 3.0, 1e-12,
               "three-point plane derives v from the third point");
    check_point(u_direction_plane->origin(), frame_origin, 1e-12,
                "point-vector-point plane preserves its origin");
    check(u_direction_plane->u_direction().approximately_equal(
              {0.0, 0.0, 1.0}, 1e-12),
          "point-vector-point plane uses the supplied positive-u direction");
    check(u_direction_plane->v_direction().approximately_equal(
              {0.0, 1.0, 0.0}, 1e-12),
          "point-vector-point plane orients v toward its point");
    check(u_direction_plane->normal().approximately_equal(
              {-1.0, 0.0, 0.0}, 1e-12),
          "point-vector-point plane produces a right-handed frame");
    check_point(spline_3d->evaluate(4.0), {1.0, 2.0, 3.0}, 1e-12,
                "make_nurbs_spline3 value");
    check_point(valarray_spline_3d->evaluate(4.0), {1.0, 2.0, 3.0}, 1e-12,
                "make_nurbs_spline3 valarray value");
    check(closed_spline_3d->is_closed(),
          "make_nurbs_spline3 forwards closure");
    check(closed_valarray_spline_3d->is_closed(),
          "3D valarray factory forwards closure");

    auto zero_vector_2d = nurbspath::make_vector2<real>();
    auto zero_point_2d = nurbspath::make_point2<real>();
    auto zero_vector_3d = nurbspath::make_vector3<real>();
    auto zero_point_3d = nurbspath::make_point3<real>();
    check(zero_vector_2d->is_near_zero() && *zero_point_2d == point2<real>::origin() &&
              zero_vector_3d->is_near_zero() && *zero_point_3d == point3<real>::origin(),
          "zero-value creator overloads");

    bool forwarded_validation = false;
    try {
        static_cast<void>(nurbspath::make_ray3(
            point3<real>::origin(), vector3<real>::zero()));
    } catch (const std::invalid_argument&) {
        forwarded_validation = true;
    }
    check(forwarded_validation, "creators preserve constructor validation");

    bool rejected_coincident_points_2d = false;
    try {
        static_cast<void>(nurbspath::make_ray2_from_points(
            point2<real>::origin(), point2<real>::origin()));
    } catch (const std::invalid_argument&) {
        rejected_coincident_points_2d = true;
    }
    check(rejected_coincident_points_2d,
          "make_ray2_from_points rejects coincident points");

    bool rejected_coincident_points_3d = false;
    try {
        static_cast<void>(nurbspath::make_ray3_from_points(
            point3<real>::origin(), point3<real>::origin()));
    } catch (const std::invalid_argument&) {
        rejected_coincident_points_3d = true;
    }
    check(rejected_coincident_points_3d,
          "make_ray3_from_points rejects coincident points");

    bool forwarded_point_tolerance = false;
    try {
        static_cast<void>(nurbspath::make_ray2_from_points(
            point2<real>::origin(), point2<real>{0.5, 0.0}, 1.0));
    } catch (const std::invalid_argument&) {
        forwarded_point_tolerance = true;
    }
    check(forwarded_point_tolerance,
          "make_ray2_from_points forwards the separation tolerance");

    bool rejected_coincident_plane_points = false;
    try {
        static_cast<void>(nurbspath::make_plane3_from_points(
            frame_origin, frame_origin, frame_plane_point));
    } catch (const std::domain_error&) {
        rejected_coincident_plane_points = true;
    }
    check(rejected_coincident_plane_points,
          "make_plane3_from_points rejects a coincident u point");

    bool rejected_collinear_plane_points = false;
    try {
        static_cast<void>(nurbspath::make_plane3_from_points(
            frame_origin,
            point3<real>{2.0, 2.0, 3.0},
            point3<real>{4.0, 2.0, 3.0}));
    } catch (const std::domain_error&) {
        rejected_collinear_plane_points = true;
    }
    check(rejected_collinear_plane_points,
          "make_plane3_from_points rejects collinear points");

    bool rejected_zero_plane_u_direction = false;
    try {
        static_cast<void>(nurbspath::make_plane3_from_u_direction(
            frame_origin, vector3<real>::zero(), frame_plane_point));
    } catch (const std::domain_error&) {
        rejected_zero_plane_u_direction = true;
    }
    check(rejected_zero_plane_u_direction,
          "make_plane3_from_u_direction rejects a zero u direction");

    bool rejected_plane_point_on_u_axis = false;
    try {
        static_cast<void>(nurbspath::make_plane3_from_u_direction(
            frame_origin,
            vector3<real>{2.0, 0.0, 0.0},
            point3<real>{4.0, 2.0, 3.0}));
    } catch (const std::domain_error&) {
        rejected_plane_point_on_u_axis = true;
    }
    check(rejected_plane_point_on_u_axis,
          "make_plane3_from_u_direction rejects a point on the u-axis");

    auto plane_with_large_tolerance =
        nurbspath::make_plane3_from_u_direction(
            frame_origin,
            vector3<real>{10.0, 0.0, 0.0},
            point3<real>{6.0, 4.0, 3.0},
            1.0);
    check(plane_with_large_tolerance->v_direction().approximately_equal(
              {0.0, 1.0, 0.0}, 1e-12),
          "plane factory applies tolerance to source geometry");

    bool rejected_plane_point_within_tolerance = false;
    try {
        static_cast<void>(nurbspath::make_plane3_from_u_direction(
            frame_origin,
            vector3<real>{10.0, 0.0, 0.0},
            point3<real>{6.0, 2.5, 3.0},
            1.0));
    } catch (const std::domain_error&) {
        rejected_plane_point_within_tolerance = true;
    }
    check(rejected_plane_point_within_tolerance,
          "plane point tolerance is independent of u-direction length");

    bool valarray_validation = false;
    try {
        static_cast<void>(nurbspath::make_nurbs_spline2<real>(
            valarray_control_points_2d,
            std::valarray<real>{1.0},
            valarray_knots_2d,
            1));
    } catch (const std::invalid_argument&) {
        valarray_validation = true;
    }
    check(valarray_validation,
          "valarray creators preserve NURBS constructor validation");
    return finish("14_test_creators");
}
