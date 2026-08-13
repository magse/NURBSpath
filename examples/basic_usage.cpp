#include <nurbspath/config.hpp>
#include <nurbspath/nurbspath.hpp>

#include <iostream>
#include <vector>

int main() {
    using real = double;
    using nurbspath::point3;

    // Measured stations retain their physical arc-length values as spline s.
    const std::vector<point3<real>> measured_points{
        {0.0, 0.0, 0.0},
        {1.0, 0.2, 0.3},
        {2.0, 0.8, 0.6},
        {3.0, 1.8, 0.2},
        {4.0, 3.0, 0.0}
    };
    const std::vector<real> arc_lengths{0.0, 1.1, 2.3, 3.6, 5.2};
    const auto path = nurbspath::nurbs_spline3<real>::interpolate(
        measured_points, arc_lengths, 3);

    const real s = 2.0;
    const auto sample = path.derivatives_at(s);
    std::cout << "path(" << s << ") = ("
              << sample.point.x() << ", "
              << sample.point.y() << ", "
              << sample.point.z() << ")\n";

    const nurbspath::plane3<real> ground(
        {0.0, 0.0, 0.0}, nurbspath::vector3<real>::unit_z());
    const auto contacts = nurbspath::intersect_spline_plane(path, ground);
    std::cout << "ground contacts: " << contacts.points.size() << '\n';

    const point3<real> query{2.0, 1.0, 1.5};
    const auto closest = nurbspath::distance_to_spline(query, path);
    std::cout << "distance to path: " << closest.distance
              << " at s=" << closest.s << '\n';
}
