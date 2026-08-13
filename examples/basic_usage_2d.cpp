#include <nurbspath/config.hpp>
#include <nurbspath/nurbspath.hpp>

#include <cstddef>
#include <iostream>
#include <vector>

int main() {
    using real = double;
    using nurbspath::point2;
    using nurbspath::vector2;

    const point2<real> ray_origin{-2.0, 0.0};
    const vector2<real> ray_direction{1.0, 0.0};
    const nurbspath::ray2<real> ray(ray_origin, ray_direction);
    const nurbspath::circle2<real> circle({0.0, 0.0}, 1.0);

    const auto contacts = nurbspath::intersect_ray_circle(ray, circle);
    std::cout << "ray-circle contacts: " << contacts.points.size() << '\n';
    for (const auto& contact : contacts.points) {
        std::cout << "  s=" << contact.s << " point=("
                  << contact.point.x() << ", "
                  << contact.point.y() << ")\n";
    }

    const std::vector<point2<real>> samples{
        {-1.0, -0.5},
        {0.0, 1.0},
        {1.0, -0.5}
    };
    const std::vector<real> stations{0.0, 1.5, 3.0};
    const auto path = nurbspath::nurbs_spline2<real>::interpolate(
        samples, stations, 2);

    const real s = 1.5;
    const point2<real> path_point = path.point_at(s);
    std::cout << "path(" << s << ") = ("
              << path_point.x() << ", " << path_point.y() << ")\n";

    const point2<real> query{0.5, 0.0};
    const auto closest = nurbspath::distance_to_spline(query, path);
    std::cout << "distance from query to path: " << closest.distance
              << " at s=" << closest.s << '\n';

    // Three periods of an open curve are connected by translating each new
    // copy so that it starts exactly where the preceding copy ended.
    const auto repeated_path = nurbspath::nurbs_spline2<real>::interpolate(
        {{0.0, 0.0},
         {1.0, 1.0},
         {2.0, 0.0},
         {3.0, -1.0},
         {4.0, 0.0}},
        {0.0, 1.0, 2.0, 3.0, 4.0},
        3,
        false,
        std::size_t{3});
    const point2<real> second_period_point = repeated_path.point_at(4.5);
    std::cout << "repeated path: periods=" << repeated_path.period_count()
              << " closed=" << repeated_path.is_closed()
              << " periodic=" << repeated_path.is_periodic()
              << " end=(" << repeated_path.get_end().x() << ", "
              << repeated_path.get_end().y() << ")"
              << " point(4.5)=(" << second_period_point.x() << ", "
              << second_period_point.y() << ")\n";
}
