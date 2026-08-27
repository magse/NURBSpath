#include <nurbspath/config.hpp>
#include <nurbspath/utility.hpp>

#include "test_support.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

template <typename EXCEPTION, typename FUNCTION>
bool throws_exception(FUNCTION&& function) {
    try {
        std::forward<FUNCTION>(function)();
    } catch (const EXCEPTION&) {
        return true;
    } catch (...) {
    }
    return false;
}

static_assert(nurbspath::nurbs_knot_count(1, 2) == 4);
static_assert(nurbspath::nurbs_knot_count(2, 3) == 6);
static_assert(nurbspath::nurbs_knot_count(2, 4) == 7);
static_assert(nurbspath::nurbs_knot_count(5, 6) == 12);
static_assert(nurbspath::nurbs_knot_count(5, 7) == 13);
static_assert(nurbspath::nurbs_knot_count(5, 10) == 16);
static_assert(
    nurbspath::nurbs_knot_count(
        1, std::numeric_limits<std::size_t>::max() - 2) ==
    std::numeric_limits<std::size_t>::max());

} // namespace

int main() {
    using test_support::check;

    for (std::size_t degree = 1; degree <= 16; ++degree) {
        for (std::size_t control_count = degree + 1;
             control_count <= degree + 32;
             ++control_count) {
            check(
                nurbspath::nurbs_knot_count(degree, control_count) ==
                    control_count + degree + 1,
                "knot count follows control count plus degree plus one");
        }
    }

    check(
        throws_exception<std::invalid_argument>([] {
            (void)nurbspath::nurbs_knot_count(0, 2);
        }),
        "zero degree is rejected");
    check(
        throws_exception<std::invalid_argument>([] {
            (void)nurbspath::nurbs_knot_count(1, 0);
        }),
        "zero control-point count is rejected");
    check(
        throws_exception<std::invalid_argument>([] {
            (void)nurbspath::nurbs_knot_count(3, 3);
        }),
        "control-point count equal to degree is rejected");
    check(
        throws_exception<std::invalid_argument>([] {
            (void)nurbspath::nurbs_knot_count(4, 3);
        }),
        "control-point count below degree is rejected");
    check(
        throws_exception<std::overflow_error>([] {
            (void)nurbspath::nurbs_knot_count(
                1, std::numeric_limits<std::size_t>::max());
        }),
        "overflowing knot count is rejected");

    return test_support::finish("20_test_knot_count");
}
