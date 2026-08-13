#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <numbers>

int main() {
    using namespace test_support;

    const vector3<real> x{1.0, 0.0, 0.0};
    const vector3<real> y{0.0, 2.0, 0.0};
    check_point(point3<real>{1.0, 2.0, 3.0} + x,
                {2.0, 2.0, 3.0}, 1e-12, "point plus vector");
    check(x.cross(y).approximately_equal({0.0, 0.0, 2.0}, 1e-12),
          "vector cross product");
    check_near(x.dot(y), 0.0, 1e-12, "vector dot product");
    check_near(y.normalized().length(), 1.0, 1e-12, "vector normalization");
    check_near(x.angle_to(y), std::numbers::pi / 2.0, 1e-12, "vector angle");
    check(x.reflected({1.0, 0.0, 0.0}).approximately_equal(
              {-1.0, 0.0, 0.0}, 1e-12),
          "vector reflection");
    return finish("01_test_vector_point");
}
