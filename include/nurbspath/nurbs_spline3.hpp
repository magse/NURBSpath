#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point3.hpp"
#include "nurbspath/utility.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace nurbspath {

template <std::floating_point REAL>
/**
 * @brief Position and first two derivatives evaluated at one spline parameter.
 * @tparam REAL Floating-point scalar type.
 */
struct spline_derivatives3 {
    point3<REAL> point; ///< Evaluated world-space position.
    vector3<REAL> first; ///< First derivative with respect to s.
    vector3<REAL> second; ///< Second derivative with respect to s.
};

/**
 * @brief Rational B-spline curve embedded in the shared 3D world system.
 *
 * The native parameter is `s`. Derivatives are computed analytically from
 * B-spline basis derivatives and the rational quotient rule.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class nurbs_spline3 {
public:
    /**
     * @brief Construct a NURBS curve from its complete definition.
     * @param control_points World-space control points.
     * @param weights Positive rational weight for every control point.
     * @param knots Nondecreasing knot vector.
     * @param degree Positive polynomial degree below control-point count.
     * @param tolerance Positive definition and parameter-boundary tolerance.
     * @throws std::invalid_argument When counts, degree, weights, knots, or tolerance are invalid.
     */
    nurbs_spline3(
        std::vector<point3<REAL>> control_points,
        std::vector<REAL> weights,
        std::vector<REAL> knots,
        std::size_t degree,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : nurbs_spline3(
              std::move(control_points),
              std::move(weights),
              std::move(knots),
              degree,
              false,
              tolerance) {}

    /**
     * @brief Construct an open or closed 3D NURBS curve.
     * @param control_points World-space control points.
     * @param weights Positive rational weight for every control point.
     * @param knots Nondecreasing knot vector.
     * @param degree Positive polynomial degree below control-point count.
     * @param closed True when the two active-domain endpoints must coincide.
     * @param tolerance Positive validation and parameter-boundary tolerance.
     * @throws std::invalid_argument When the definition or closed seam is invalid.
     */
    nurbs_spline3(
        std::vector<point3<REAL>> control_points,
        std::vector<REAL> weights,
        std::vector<REAL> knots,
        std::size_t degree,
        bool closed,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : control_points_(std::move(control_points)),
          weights_(std::move(weights)),
          knots_(std::move(knots)),
          degree_(degree),
          tolerance_(tolerance),
          closed_(closed) {
        validate_definition();
        const spline_derivatives3<REAL> start_values = derivatives_at(s_min());
        const spline_derivatives3<REAL> end_values = derivatives_at(s_max());
        start_ = start_values.point;
        end_ = end_values.point;
        if (closed_ && distance(start_, end_) > closure_tolerance()) {
            throw std::invalid_argument("closed NURBS endpoints must coincide");
        }
    }

    /** @brief Get all control points. @return Constant control-point vector reference. */
    [[nodiscard]] const std::vector<point3<REAL>>& control_points() const noexcept {
        return control_points_;
    }
    /** @brief Get all rational weights. @return Constant weight vector reference. */
    [[nodiscard]] const std::vector<REAL>& weights() const noexcept { return weights_; }
    /** @brief Get the full knot vector. @return Constant knot vector reference. */
    [[nodiscard]] const std::vector<REAL>& knots() const noexcept { return knots_; }
    /** @brief Get the polynomial degree. @return Spline degree. */
    [[nodiscard]] std::size_t degree() const noexcept { return degree_; }

    /** @brief Get the spline definition tolerance. @return Positive tolerance. */
    [[nodiscard]] REAL tolerance() const noexcept { return tolerance_; }

    /** @brief Report whether the spline has a closed seam. @return True for closed curves. */
    [[nodiscard]] bool is_closed() const noexcept { return closed_; }

    /**
     * @brief Get the cached point at the start of the active domain.
     * @return Constant reference to the point at `s_min()` without evaluation.
     */
    [[nodiscard]] const point3<REAL>& get_start() const noexcept { return start_; }

    /**
     * @brief Get the cached point at the end of the active domain.
     * @return Constant reference to the point at `s_max()` without evaluation.
     */
    [[nodiscard]] const point3<REAL>& get_end() const noexcept { return end_; }

    /** @brief Get the lower active parameter bound. @return Minimum valid s. */
    [[nodiscard]] REAL s_min() const noexcept { return knots_[degree_]; }
    /** @brief Get the upper active parameter bound. @return Maximum valid s. */
    [[nodiscard]] REAL s_max() const noexcept { return knots_[control_points_.size()]; }

    /**
     * @brief Evaluate position and the first two derivatives together.
     * @param s Finite parameter in the active knot domain.
     * @return Position, first derivative, and second derivative.
     * @throws std::out_of_range When s is non-finite or lies outside a
     * configured path's tolerated active domain.
     * @throws std::domain_error When the homogeneous weight is near zero.
     */
    [[nodiscard]] spline_derivatives3<REAL> derivatives_at(REAL s) const {
        const auto derivatives = rational_derivatives_at(s, 2);

        return {
            point3<REAL>{
                derivatives[0].x, derivatives[0].y, derivatives[0].z},
            derivatives[1],
            derivatives[2]
        };
    }

    /**
     * @brief Evaluate the curve position.
     * @param s Parameter in the active knot domain.
     * @return World-space curve point.
     */
    [[nodiscard]] point3<REAL> evaluate(REAL s) const {
        return derivatives_at(s).point;
    }

    /**
     * @brief Evaluate the curve position.
     * @param s Parameter in the active knot domain.
     * @return World-space curve point.
     */
    [[nodiscard]] point3<REAL> point_at(REAL s) const {
        return evaluate(s);
    }

    /**
     * @brief Evaluate the first derivative.
     * @param s Parameter in the active knot domain.
     * @return Derivative with respect to s.
     */
    [[nodiscard]] vector3<REAL> first_derivative(REAL s) const {
        return derivatives_at(s).first;
    }

    /**
     * @brief Evaluate the second derivative.
     * @param s Parameter in the active knot domain.
     * @return Second derivative with respect to s.
     */
    [[nodiscard]] vector3<REAL> second_derivative(REAL s) const {
        return derivatives_at(s).second;
    }

    /**
     * @brief Evaluate the third analytic rational derivative.
     *
     * Homogeneous derivatives above the polynomial degree are zero, but the
     * rational quotient terms can still produce a nonzero third derivative.
     * At an internal knot without third-order continuity, the result is the
     * right-hand span derivative; `s_max()` uses the left-hand span.
     *
     * @param s Finite parameter in the active knot domain.
     * @return Third derivative with respect to s.
     * @throws std::out_of_range When s is non-finite or lies outside the
     * configured path's tolerated active domain.
     * @throws std::domain_error When the homogeneous weight is near zero.
     */
    [[nodiscard]] vector3<REAL> third_derivative(REAL s) const {
        return rational_derivatives_at(s, 3)[3];
    }

    /**
     * @brief Evaluate a unit tangent.
     * @param s Parameter in the active knot domain.
     * @param tolerance Minimum accepted first-derivative length.
     * @return Normalized first derivative.
     * @throws std::domain_error When the first derivative is too small.
     */
    [[nodiscard]] vector3<REAL> tangent(
        REAL s,
        REAL tolerance = vector3<REAL>::default_tolerance()) const {
        return first_derivative(s).normalized(tolerance);
    }

    /**
     * @brief Estimate total arc length using a uniform-s polyline.
     * @param segment_count Positive number of straight approximation segments.
     * @return Approximate world-space arc length.
     * @throws std::invalid_argument When segment_count is zero.
     */
    [[nodiscard]] REAL approximate_arc_length(std::size_t segment_count = 512) const {
        if (segment_count == 0) {
            throw std::invalid_argument("segment_count must be positive");
        }
        REAL length = REAL(0);
        point3<REAL> previous = evaluate(s_min());
        for (std::size_t index = 1; index <= segment_count; ++index) {
            const REAL fraction = static_cast<REAL>(index) /
                                  static_cast<REAL>(segment_count);
            const REAL s = s_min() + fraction * (s_max() - s_min());
            const point3<REAL> current = evaluate(s);
            length += distance(previous, current);
            previous = current;
        }
        return length;
    }

    /**
     * @brief Build a unit-weight B-spline interpolating measured samples.
     *
     * Original strictly increasing arc-length station values become the native
     * `s` domain. Averaged clamped knots and global interpolation are used.
     *
     * @param samples World-space positions to interpolate exactly.
     * @param arc_length_parameters Strictly increasing station for each sample.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param tolerance Positive solve and spline-definition tolerance.
     * @return Interpolating NURBS curve with unit weights.
     * @throws std::invalid_argument When sample data or parameters are invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    [[nodiscard]] static nurbs_spline3 interpolate(
        const std::vector<point3<REAL>>& samples,
        const std::vector<REAL>& arc_length_parameters,
        std::size_t requested_degree = 3,
        REAL tolerance = REAL(1e-10)) {
        return interpolate(
            samples,
            arc_length_parameters,
            requested_degree,
            false,
            tolerance);
    }

    /**
     * @brief Build an open or closed unit-weight interpolating 3D spline.
     * @param samples World-space positions to interpolate exactly.
     * @param arc_length_parameters Strictly increasing station for each sample.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param closed True to require coincident active-domain endpoints.
     * @param tolerance Positive solve, seam, and definition tolerance.
     * @return Interpolating 3D NURBS curve with unit weights.
     * @throws std::invalid_argument When sample data or closure is invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    [[nodiscard]] static nurbs_spline3 interpolate(
        const std::vector<point3<REAL>>& samples,
        const std::vector<REAL>& arc_length_parameters,
        std::size_t requested_degree,
        bool closed,
        REAL tolerance = REAL(1e-10)) {
        if (samples.size() != arc_length_parameters.size()) {
            throw std::invalid_argument("samples and arc-length parameters must have equal size");
        }
        if (samples.size() < 2) {
            throw std::invalid_argument("at least two samples are required");
        }
        if (requested_degree == 0) {
            throw std::invalid_argument("interpolation degree must be positive");
        }
        if (!(tolerance > REAL(0)) || !std::isfinite(tolerance)) {
            throw std::invalid_argument(
                "interpolation tolerance must be finite and positive");
        }
        for (std::size_t index = 1; index < arc_length_parameters.size(); ++index) {
            if (!(arc_length_parameters[index] > arc_length_parameters[index - 1])) {
                throw std::invalid_argument("arc-length parameters must be strictly increasing");
            }
        }

        if (closed && distance(samples.front(), samples.back()) > tolerance) {
            throw std::invalid_argument(
                "closed interpolation requires coincident first and final samples");
        }

        const std::size_t point_count = samples.size();
        const std::size_t degree = std::min(requested_degree, point_count - 1);
        const std::size_t n = point_count - 1;
        std::vector<REAL> knots(point_count + degree + 1, REAL(0));

        std::fill_n(knots.begin(), degree + 1, arc_length_parameters.front());
        std::fill_n(
            knots.end() - static_cast<std::ptrdiff_t>(degree + 1),
            degree + 1,
            arc_length_parameters.back());

        // Knot averaging is the standard stable choice for global B-spline
        // interpolation.  There are n-degree internal knots.
        for (std::size_t j = 1; j <= n - degree; ++j) {
            REAL sum = REAL(0);
            for (std::size_t index = j; index < j + degree; ++index) {
                sum += arc_length_parameters[index];
            }
            knots[j + degree] = sum / static_cast<REAL>(degree);
        }

        std::vector<REAL> unit_weights(point_count, REAL(1));
        // The seed object supplies span and basis evaluation. Its current
        // control coordinates do not affect the interpolation matrix.
        const nurbs_spline3 seed(samples, unit_weights, knots, degree, tolerance);
        std::vector<std::vector<REAL>> matrix(
            point_count, std::vector<REAL>(point_count, REAL(0)));
        for (std::size_t row = 0; row < point_count; ++row) {
            const REAL s = arc_length_parameters[row];
            const std::size_t span = seed.find_span(s);
            const auto values = seed.basis_function_derivatives(span, s, 0).front();
            for (std::size_t local = 0; local <= degree; ++local) {
                matrix[row][span - degree + local] = values[local];
            }
        }

        std::vector<REAL> x_values(point_count);
        std::vector<REAL> y_values(point_count);
        std::vector<REAL> z_values(point_count);
        for (std::size_t index = 0; index < point_count; ++index) {
            x_values[index] = samples[index].x;
            y_values[index] = samples[index].y;
            z_values[index] = samples[index].z;
        }
        const auto x_controls = solve_linear_system(matrix, x_values, tolerance);
        const auto y_controls = solve_linear_system(matrix, y_values, tolerance);
        const auto z_controls = solve_linear_system(matrix, z_values, tolerance);

        std::vector<point3<REAL>> control_points(point_count);
        for (std::size_t index = 0; index < point_count; ++index) {
            control_points[index] = {
                x_controls[index], y_controls[index], z_controls[index]};
        }
        return nurbs_spline3(
            std::move(control_points), std::move(unit_weights),
            std::move(knots), degree, closed, tolerance);
    }

    /**
     * @brief Replace this curve with an interpolant through measured samples.
     * @param samples World-space positions to interpolate exactly.
     * @param arc_length_parameters Strictly increasing station for each sample.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param tolerance Positive solve and spline-definition tolerance.
     * @throws std::invalid_argument When sample data or parameters are invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    void adopt_to_points(
        const std::vector<point3<REAL>>& samples,
        const std::vector<REAL>& arc_length_parameters,
        std::size_t requested_degree = 3,
        REAL tolerance = REAL(1e-10)) {
        *this = interpolate(
            samples, arc_length_parameters, requested_degree, tolerance);
    }

    /**
     * @brief Replace this curve with an open or closed interpolant.
     * @param samples World-space positions to interpolate exactly.
     * @param arc_length_parameters Strictly increasing station for each sample.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param closed True to require coincident active-domain endpoints.
     * @param tolerance Positive solve, seam, and definition tolerance.
     * @throws std::invalid_argument When sample data or closure is invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    void adopt_to_points(
        const std::vector<point3<REAL>>& samples,
        const std::vector<REAL>& arc_length_parameters,
        std::size_t requested_degree,
        bool closed,
        REAL tolerance = REAL(1e-10)) {
        *this = interpolate(
            samples,
            arc_length_parameters,
            requested_degree,
            closed,
            tolerance);
    }

private:
    void validate_definition() const {
        if (!(tolerance_ > REAL(0)) || !std::isfinite(tolerance_)) {
            throw std::invalid_argument("NURBS tolerance must be finite and positive");
        }
        if (degree_ == 0) {
            throw std::invalid_argument("NURBS degree must be positive");
        }
        if (control_points_.size() <= degree_) {
            throw std::invalid_argument("NURBS requires more control points than its degree");
        }
        if (weights_.size() != control_points_.size()) {
            throw std::invalid_argument("NURBS weights and control points must have equal size");
        }
        if (knots_.size() != control_points_.size() + degree_ + 1) {
            throw std::invalid_argument("NURBS knot count must equal control count + degree + 1");
        }
        for (const point3<REAL>& point : control_points_) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) {
                throw std::invalid_argument("NURBS control points must be finite");
            }
        }
        for (REAL weight : weights_) {
            if (!(weight > REAL(0)) || !std::isfinite(weight)) {
                throw std::invalid_argument("NURBS weights must be finite and positive");
            }
        }
        for (std::size_t index = 0; index < knots_.size(); ++index) {
            if (!std::isfinite(knots_[index])) {
                throw std::invalid_argument("NURBS knots must be finite");
            }
            if (index > 0 && knots_[index] < knots_[index - 1]) {
                throw std::invalid_argument("NURBS knots must be nondecreasing");
            }
        }
        if (!(s_max() > s_min())) {
            throw std::invalid_argument("NURBS active parameter domain must have positive length");
        }
    }

    [[nodiscard]] REAL closure_tolerance() const noexcept {
        REAL scale = REAL(1);
        for (const point3<REAL>& point : control_points_) {
            scale = std::max({
                scale,
                std::abs(point.x),
                std::abs(point.y),
                std::abs(point.z)});
        }
        return REAL(16) * tolerance_ * scale;
    }

    [[nodiscard]] REAL checked_parameter(REAL s) const {
        if (!std::isfinite(s)) {
            throw std::out_of_range("NURBS parameter s must be finite");
        }
        if (s < s_min() - tolerance_ || s > s_max() + tolerance_) {
            throw std::out_of_range("NURBS parameter s is outside the active knot domain");
        }
        return std::clamp(s, s_min(), s_max());
    }

    [[nodiscard]] std::size_t find_span(REAL s) const noexcept {
        const std::size_t n = control_points_.size() - 1;
        if (s >= knots_[n + 1]) {
            return n;
        }
        if (s <= knots_[degree_]) {
            return degree_;
        }

        std::size_t lower = degree_;
        std::size_t upper = n + 1;
        std::size_t middle = (lower + upper) / 2;
        while (s < knots_[middle] || s >= knots_[middle + 1]) {
            if (s < knots_[middle]) {
                upper = middle;
            } else {
                lower = middle;
            }
            middle = (lower + upper) / 2;
        }
        return middle;
    }

    /// Algorithm A2.3 from The NURBS Book.
    [[nodiscard]] std::vector<std::vector<REAL>> basis_function_derivatives(
        std::size_t span,
        REAL s,
        std::size_t order) const {
        order = std::min(order, degree_);
        const std::size_t p = degree_;
        std::vector<std::vector<REAL>> ndu(
            p + 1, std::vector<REAL>(p + 1, REAL(0)));
        std::vector<REAL> left(p + 1, REAL(0));
        std::vector<REAL> right(p + 1, REAL(0));
        ndu[0][0] = REAL(1);

        for (std::size_t j = 1; j <= p; ++j) {
            left[j] = s - knots_[span + 1 - j];
            right[j] = knots_[span + j] - s;
            REAL saved = REAL(0);
            for (std::size_t r = 0; r < j; ++r) {
                ndu[j][r] = right[r + 1] + left[j - r];
                const REAL temporary = ndu[r][j - 1] / ndu[j][r];
                ndu[r][j] = saved + right[r + 1] * temporary;
                saved = left[j - r] * temporary;
            }
            ndu[j][j] = saved;
        }

        std::vector<std::vector<REAL>> derivatives(
            order + 1, std::vector<REAL>(p + 1, REAL(0)));
        for (std::size_t j = 0; j <= p; ++j) {
            derivatives[0][j] = ndu[j][p];
        }

        std::vector<std::vector<REAL>> work(
            2, std::vector<REAL>(p + 1, REAL(0)));
        for (std::size_t r = 0; r <= p; ++r) {
            std::size_t source = 0;
            std::size_t destination = 1;
            work[0][0] = REAL(1);

            for (std::size_t k = 1; k <= order; ++k) {
                REAL derivative = REAL(0);
                const auto signed_r = static_cast<std::ptrdiff_t>(r);
                const auto signed_k = static_cast<std::ptrdiff_t>(k);
                const auto rk = signed_r - signed_k;
                const auto pk = static_cast<std::ptrdiff_t>(p - k);

                if (r >= k) {
                    work[destination][0] = work[source][0] /
                        ndu[static_cast<std::size_t>(pk + 1)][static_cast<std::size_t>(rk)];
                    derivative = work[destination][0] *
                        ndu[static_cast<std::size_t>(rk)][static_cast<std::size_t>(pk)];
                }

                const std::ptrdiff_t j1 = rk >= -1 ? 1 : -rk;
                const std::ptrdiff_t j2 =
                    signed_r - 1 <= pk ? signed_k - 1 :
                    static_cast<std::ptrdiff_t>(p) - signed_r;
                for (std::ptrdiff_t j = j1; j <= j2; ++j) {
                    work[destination][static_cast<std::size_t>(j)] =
                        (work[source][static_cast<std::size_t>(j)] -
                         work[source][static_cast<std::size_t>(j - 1)]) /
                        ndu[static_cast<std::size_t>(pk + 1)]
                           [static_cast<std::size_t>(rk + j)];
                    derivative += work[destination][static_cast<std::size_t>(j)] *
                        ndu[static_cast<std::size_t>(rk + j)]
                           [static_cast<std::size_t>(pk)];
                }

                if (signed_r <= pk) {
                    work[destination][k] = -work[source][k - 1] /
                        ndu[static_cast<std::size_t>(pk + 1)][r];
                    derivative += work[destination][k] * ndu[r][static_cast<std::size_t>(pk)];
                }
                derivatives[k][r] = derivative;
                std::swap(source, destination);
            }
        }

        // The recurrence above omits the p!/(p-k)! scale.
        REAL scale = static_cast<REAL>(p);
        for (std::size_t k = 1; k <= order; ++k) {
            for (std::size_t j = 0; j <= p; ++j) {
                derivatives[k][j] *= scale;
            }
            scale *= static_cast<REAL>(p - k);
        }
        return derivatives;
    }

    [[nodiscard]] std::array<vector3<REAL>, 4> rational_derivatives_at(
        REAL s,
        std::size_t requested_order) const {
        requested_order = std::min<std::size_t>(requested_order, 3);
        const REAL parameter = checked_parameter(s);
        const std::size_t span = find_span(parameter);
        const std::size_t basis_order = std::min(requested_order, degree_);
        const auto basis_derivatives = basis_function_derivatives(
            span, parameter, basis_order);

        // Each homogeneous derivative contains the numerator vector A^(k) and
        // the scalar weight derivative w^(k). Orders above the degree remain
        // zero while the rational quotient recurrence continues through the
        // requested order.
        std::array<vector3<REAL>, 4> numerator{};
        std::array<REAL, 4> weight_derivative{};
        for (std::size_t order = 0; order <= basis_order; ++order) {
            for (std::size_t local = 0; local <= degree_; ++local) {
                const std::size_t control_index = span - degree_ + local;
                const REAL coefficient =
                    basis_derivatives[order][local] * weights_[control_index];
                const point3<REAL>& control = control_points_[control_index];
                numerator[order] += coefficient * vector3<REAL>{
                    control.x, control.y, control.z};
                weight_derivative[order] += coefficient;
            }
        }

        if (std::abs(weight_derivative[0]) <= tolerance_) {
            throw std::domain_error("NURBS homogeneous weight is near zero");
        }

        std::array<vector3<REAL>, 4> result{};
        result[0] = numerator[0] / weight_derivative[0];
        for (std::size_t order = 1; order <= requested_order; ++order) {
            vector3<REAL> value = numerator[order];
            REAL binomial = REAL(1);
            for (std::size_t weight_order = 1;
                 weight_order <= order;
                 ++weight_order) {
                binomial *=
                    static_cast<REAL>(order + 1 - weight_order) /
                    static_cast<REAL>(weight_order);
                value -= binomial * weight_derivative[weight_order] *
                         result[order - weight_order];
            }
            result[order] = value / weight_derivative[0];
        }
        return result;
    }

    std::vector<point3<REAL>> control_points_;
    std::vector<REAL> weights_;
    std::vector<REAL> knots_;
    std::size_t degree_;
    REAL tolerance_;
    bool closed_ = false;
    point3<REAL> start_{};
    point3<REAL> end_{};
};

} // namespace nurbspath
