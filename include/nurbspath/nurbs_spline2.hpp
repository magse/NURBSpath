#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point2.hpp"
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

/**
 * @brief Position and first two derivatives at one 2D spline parameter.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct spline_derivatives2 {
    point2<REAL> point; ///< Evaluated position in the 2D world.
    vector2<REAL> first; ///< First derivative with respect to s.
    vector2<REAL> second; ///< Second derivative with respect to s.
};

/**
 * @brief Rational B-spline curve in the independent 2D world.
 *
 * The native parameter is `s`. Position and analytic rational derivatives are
 * evaluated entirely in two dimensions. Use `project(plane, spline)` to create
 * a separate 3D spline by projecting only this curve's control points. A
 * repeated spline evaluates translated copies of its fundamental period so
 * that every copy starts where the preceding copy ends.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class nurbs_spline2 {
public:
    /**
     * @brief Construct a 2D NURBS curve from its complete definition.
     * @param control_points_value Control points in the 2D world.
     * @param weights_value Positive rational weight for every control point.
     * @param knots_value Nondecreasing knot vector.
     * @param degree_value Positive degree below the control-point count.
     * @param tolerance Positive definition and parameter-boundary tolerance.
     * @throws std::invalid_argument When counts, degree, weights, knots, or tolerance are invalid.
     */
    nurbs_spline2(
        std::vector<point2<REAL>> control_points_value,
        std::vector<REAL> weights_value,
        std::vector<REAL> knots_value,
        std::size_t degree_value,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : nurbs_spline2(
              std::move(control_points_value),
              std::move(weights_value),
              std::move(knots_value),
              degree_value,
              false,
              std::size_t(1),
              tolerance) {}

    /**
     * @brief Construct a closed or repeated 2D NURBS curve.
     * @param control_points_value Control points in the 2D world.
     * @param weights_value Positive rational weight for every control point.
     * @param knots_value Nondecreasing knot vector.
     * @param degree_value Positive degree below the control-point count.
     * @param closed True when the two fundamental-period endpoints must coincide.
     * @param period_count Number of connected periods. Zero means unlimited,
     * one selects an ordinary non-repeated spline, and values above one select
     * a finite repeated path.
     * @param tolerance Positive validation and parameter-boundary tolerance.
     * @throws std::invalid_argument When the definition, repeated seam, or
     * finite repeated domain is invalid.
     */
    nurbs_spline2(
        std::vector<point2<REAL>> control_points_value,
        std::vector<REAL> weights_value,
        std::vector<REAL> knots_value,
        std::size_t degree_value,
        bool closed,
        std::size_t period_count,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : control_points_(std::move(control_points_value)),
          weights_(std::move(weights_value)),
          knots_(std::move(knots_value)),
          degree_(degree_value),
          tolerance_(tolerance),
          closed_(closed),
          period_count_(period_count) {
        validate_definition();
        validate_repeated_definition();
        const spline_derivatives2<REAL> start_values =
            derivatives_at_base(period_s_min());
        const spline_derivatives2<REAL> period_end_values =
            derivatives_at_base(period_s_max());
        start_ = start_values.point;
        period_displacement_ = period_end_values.point - start_;
        if (closed_ && period_displacement_.length() > cyclic_tolerance()) {
            throw std::invalid_argument(
                "closed 2D NURBS endpoints must coincide");
        }
        const REAL first_tolerance = REAL(16) * tolerance_ * std::max({
            REAL(1),
            start_values.first.length(),
            period_end_values.first.length()});
        if (is_periodic() && degree_ >= 2 &&
            (start_values.first - period_end_values.first).length() >
                first_tolerance) {
            throw std::invalid_argument(
                "periodic 2D NURBS first derivatives must match at the seam");
        }
        const REAL second_tolerance = REAL(16) * tolerance_ * std::max({
            REAL(1),
            start_values.second.length(),
            period_end_values.second.length()});
        if (is_periodic() && degree_ >= 3 &&
            (start_values.second - period_end_values.second).length() >
                second_tolerance) {
            throw std::invalid_argument(
                "periodic 2D NURBS second derivatives must match at the seam");
        }
        if (period_count_ != 0 && !std::isfinite(s_max())) {
            throw std::invalid_argument(
                "finite repeated 2D NURBS domain is not representable");
        }
        if (period_count_ != 0) {
            end_ = period_end_values.point +
                static_cast<REAL>(period_count_ - 1) * period_displacement_;
            if (!std::isfinite(end_.x()) || !std::isfinite(end_.y())) {
                throw std::invalid_argument(
                    "finite repeated 2D NURBS endpoint is not representable");
            }
        }
    }

    /** @brief Get all 2D control points. @return Constant control-point vector reference. */
    [[nodiscard]] const std::vector<point2<REAL>>& control_points() const noexcept {
        return control_points_;
    }

    /** @brief Get all rational weights. @return Constant weight vector reference. */
    [[nodiscard]] const std::vector<REAL>& weights() const noexcept { return weights_; }

    /** @brief Get the complete knot vector. @return Constant knot vector reference. */
    [[nodiscard]] const std::vector<REAL>& knots() const noexcept { return knots_; }

    /** @brief Get the polynomial degree. @return Spline degree. */
    [[nodiscard]] std::size_t degree() const noexcept { return degree_; }

    /** @brief Get the spline definition tolerance. @return Positive tolerance. */
    [[nodiscard]] REAL tolerance() const noexcept { return tolerance_; }

    /** @brief Report whether the spline has a closed seam. @return True for closed curves. */
    [[nodiscard]] bool is_closed() const noexcept { return closed_; }

    /** @brief Report whether the path contains repeated periods. @return True unless period_count() is one. */
    [[nodiscard]] bool is_periodic() const noexcept { return period_count_ != 1; }

    /** @brief Get the configured period count. @return Zero for unlimited repetition, one for an ordinary path, or the finite period count. */
    [[nodiscard]] std::size_t period_count() const noexcept { return period_count_; }

    /**
     * @brief Get the cached point at the start of the active domain.
     * @return Constant reference to the point at `s_min()` without evaluation.
     */
    [[nodiscard]] const point2<REAL>& get_start() const noexcept { return start_; }

    /**
     * @brief Get the cached point at the end of a finite active domain.
     * @return Constant reference to the point at `s_max()` without evaluation.
     * @throws std::domain_error When period_count() is zero because an
     * unlimited path has no end.
     */
    [[nodiscard]] const point2<REAL>& get_end() const {
        if (period_count_ == 0) {
            throw std::domain_error("an unlimited 2D NURBS path has no end");
        }
        return end_;
    }

    /** @brief Get the lower active parameter bound. @return Minimum valid s. */
    [[nodiscard]] REAL s_min() const noexcept { return period_s_min(); }

    /** @brief Get the upper active parameter bound. @return Maximum valid s, or positive infinity for unlimited repetition. */
    [[nodiscard]] REAL s_max() const noexcept {
        if (period_count_ == 0) {
            return std::numeric_limits<REAL>::infinity();
        }
        return period_s_min() +
            static_cast<REAL>(period_count_) * period_length();
    }

    /** @brief Get the end parameter of the fundamental period. @return Fundamental-period upper bound. */
    [[nodiscard]] REAL period_s_max() const noexcept {
        return knots_[control_points_.size()];
    }

    /** @brief Get the fundamental parameter-period length. @return Positive period length in s units. */
    [[nodiscard]] REAL period_length() const noexcept {
        return period_s_max() - period_s_min();
    }

    /**
     * @brief Evaluate position and the first two analytic derivatives.
     * @param s Finite parameter in the configured forward domain.
     * @return Position, first derivative, and second derivative in 2D.
     * @throws std::out_of_range When s is non-finite or lies outside a
     * configured path's tolerated active domain.
     * @throws std::domain_error When the homogeneous weight is near zero.
     */
    [[nodiscard]] spline_derivatives2<REAL> derivatives_at(REAL s) const {
        const parameter_mapping mapping = checked_parameter(s);
        spline_derivatives2<REAL> result =
            derivatives_at_base(mapping.local_parameter);
        result.point += mapping.period_index * period_displacement_;
        return result;
    }

private:
    struct parameter_mapping {
        REAL local_parameter;
        REAL period_index;
    };

    [[nodiscard]] spline_derivatives2<REAL> derivatives_at_base(
        REAL parameter) const {
        const std::size_t span = find_span(parameter);
        const std::size_t requested_order = std::min<std::size_t>(2, degree_);
        const auto derivatives = basis_function_derivatives(
            span, parameter, requested_order);

        std::array<vector2<REAL>, 3> numerator{};
        std::array<REAL, 3> weight_derivative{};
        for (std::size_t order = 0; order <= requested_order; ++order) {
            for (std::size_t local = 0; local <= degree_; ++local) {
                const std::size_t control_index = span - degree_ + local;
                const REAL coefficient =
                    derivatives[order][local] * weights_[control_index];
                const point2<REAL>& control = control_points_[control_index];
                numerator[order] += coefficient *
                    vector2<REAL>{control.x(), control.y()};
                weight_derivative[order] += coefficient;
            }
        }

        if (std::abs(weight_derivative[0]) <= tolerance_) {
            throw std::domain_error("2D NURBS homogeneous weight is near zero");
        }

        const vector2<REAL> position_vector = numerator[0] / weight_derivative[0];
        vector2<REAL> first{};
        vector2<REAL> second{};
        if (degree_ >= 1) {
            first = (numerator[1] - weight_derivative[1] * position_vector) /
                    weight_derivative[0];
        }
        if (degree_ >= 2) {
            second = (numerator[2] - REAL(2) * weight_derivative[1] * first -
                      weight_derivative[2] * position_vector) /
                     weight_derivative[0];
        }

        return {
            point2<REAL>{position_vector.x(), position_vector.y()},
            first,
            second};
    }

public:

    /**
     * @brief Evaluate the curve position.
     * @param s Parameter in the active knot domain.
     * @return Curve point in the 2D world.
     */
    [[nodiscard]] point2<REAL> evaluate(REAL s) const {
        return derivatives_at(s).point;
    }

    /**
     * @brief Evaluate the curve position.
     * @param s Parameter in the active knot domain.
     * @return Curve point in the 2D world.
     */
    [[nodiscard]] point2<REAL> point_at(REAL s) const { return evaluate(s); }

    /**
     * @brief Evaluate the first derivative.
     * @param s Parameter in the active knot domain.
     * @return Derivative with respect to s in 2D.
     */
    [[nodiscard]] vector2<REAL> first_derivative(REAL s) const {
        return derivatives_at(s).first;
    }

    /**
     * @brief Evaluate the second derivative.
     * @param s Parameter in the active knot domain.
     * @return Second derivative with respect to s in 2D.
     */
    [[nodiscard]] vector2<REAL> second_derivative(REAL s) const {
        return derivatives_at(s).second;
    }

    /**
     * @brief Evaluate a unit tangent.
     * @param s Parameter in the active knot domain.
     * @param tolerance Minimum accepted first-derivative length.
     * @return Normalized first derivative.
     * @throws std::domain_error When the first derivative is too small.
     */
    [[nodiscard]] vector2<REAL> tangent(
        REAL s,
        REAL tolerance = vector2<REAL>::default_tolerance()) const {
        return first_derivative(s).normalized(tolerance);
    }

    /**
     * @brief Estimate 2D arc length using a uniform-s polyline.
     * @param segment_count Positive number of straight approximation segments.
     * @return Approximate length in 2D world units.
     * @throws std::invalid_argument When segment_count is zero.
     * @throws std::domain_error When the path repeats without a finite limit.
     */
    [[nodiscard]] REAL approximate_arc_length(std::size_t segment_count = 512) const {
        if (segment_count == 0) {
            throw std::invalid_argument("segment_count must be positive");
        }
        if (period_count_ == 0) {
            throw std::domain_error(
                "cannot approximate the total length of an unlimited 2D NURBS path");
        }
        REAL length = REAL(0);
        point2<REAL> previous = evaluate(s_min());
        for (std::size_t index = 1; index <= segment_count; ++index) {
            const REAL fraction =
                static_cast<REAL>(index) / static_cast<REAL>(segment_count);
            const REAL s = s_min() + fraction * (s_max() - s_min());
            const point2<REAL> current = evaluate(s);
            length += distance(previous, current);
            previous = current;
        }
        return length;
    }

    /**
     * @brief Build a unit-weight B-spline interpolating measured 2D samples.
     *
     * The strictly increasing arc-length stations remain the native `s`
     * values. Averaged clamped knots and global interpolation are used.
     *
     * @param samples Positions in the 2D world to interpolate exactly.
     * @param arc_length_parameters Strictly increasing station for every sample.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param tolerance Positive solve and spline-definition tolerance.
     * @return Interpolating 2D NURBS curve with unit weights.
     * @throws std::invalid_argument When sample data or parameters are invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    [[nodiscard]] static nurbs_spline2 interpolate(
        const std::vector<point2<REAL>>& samples,
        const std::vector<REAL>& arc_length_parameters,
        std::size_t requested_degree = 3,
        REAL tolerance = REAL(1e-10)) {
        return interpolate(
            samples,
            arc_length_parameters,
            requested_degree,
            false,
            std::size_t(1),
            tolerance);
    }

    /**
     * @brief Build a closed or repeated unit-weight interpolating 2D spline.
     *
     * The final station defines the fundamental period. Repeated interpolation
     * uses a cyclic uniform knot vector and translates each subsequent period
     * by the displacement from the first sample to the final sample. Closed
     * input therefore repeats its first sample at the final station.
     *
     * @param samples Positions in one fundamental period, including both ends.
     * @param arc_length_parameters Strictly increasing station for every sample.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param closed True to require coincident active-domain endpoints.
     * @param period_count Number of connected periods. Zero means unlimited,
     * one selects ordinary interpolation, and values above one select a finite
     * repeated path.
     * @param tolerance Positive solve, seam, and definition tolerance.
     * @return Interpolating 2D NURBS curve with unit weights.
     * @throws std::invalid_argument When sample data or repeated input is invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    [[nodiscard]] static nurbs_spline2 interpolate(
        const std::vector<point2<REAL>>& samples,
        const std::vector<REAL>& arc_length_parameters,
        std::size_t requested_degree,
        bool closed,
        std::size_t period_count,
        REAL tolerance = REAL(1e-10)) {
        if (samples.size() != arc_length_parameters.size()) {
            throw std::invalid_argument(
                "samples and arc-length parameters must have equal size");
        }
        if (samples.size() < 2) {
            throw std::invalid_argument("at least two samples are required");
        }
        if (requested_degree == 0) {
            throw std::invalid_argument("interpolation degree must be positive");
        }
        if (!(tolerance > REAL(0)) || !std::isfinite(tolerance)) {
            throw std::invalid_argument("interpolation tolerance must be finite and positive");
        }
        for (std::size_t index = 0; index < arc_length_parameters.size(); ++index) {
            if (!std::isfinite(arc_length_parameters[index])) {
                throw std::invalid_argument("arc-length parameters must be finite");
            }
            if (index > 0 &&
                !(arc_length_parameters[index] > arc_length_parameters[index - 1])) {
                throw std::invalid_argument(
                    "arc-length parameters must be strictly increasing");
            }
        }

        if (closed && distance(samples.front(), samples.back()) > tolerance) {
            throw std::invalid_argument(
                "closed 2D interpolation must repeat its first sample at the end");
        }

        if (period_count != 1) {
            const std::size_t unique_count = samples.size() - 1;
            if (unique_count < 2) {
                throw std::invalid_argument(
                    "periodic 2D interpolation requires at least two unique samples");
            }
            const std::size_t degree =
                std::min(requested_degree, unique_count - 1);
            const REAL s_begin = arc_length_parameters.front();
            const REAL s_end = arc_length_parameters.back();
            const REAL period = s_end - s_begin;
            const vector2<REAL> displacement = samples.back() - samples.front();
            const REAL knot_step = period / static_cast<REAL>(unique_count);
            const std::size_t control_count = unique_count + degree;

            std::vector<REAL> knots(unique_count + 2 * degree + 1);
            for (std::size_t index = 0; index < knots.size(); ++index) {
                const auto offset = static_cast<std::ptrdiff_t>(index) -
                                    static_cast<std::ptrdiff_t>(degree);
                knots[index] = s_begin + static_cast<REAL>(offset) * knot_step;
            }

            std::vector<point2<REAL>> seed_controls(control_count);
            for (std::size_t index = 0; index < control_count; ++index) {
                seed_controls[index] = samples[index % unique_count];
                if (index >= unique_count) {
                    seed_controls[index] += displacement;
                }
            }
            std::vector<REAL> unit_weights(control_count, REAL(1));
            const nurbs_spline2 seed(
                seed_controls, unit_weights, knots, degree, tolerance);

            std::vector<std::vector<REAL>> matrix(
                unique_count, std::vector<REAL>(unique_count, REAL(0)));
            for (std::size_t row = 0; row < unique_count; ++row) {
                const REAL s = arc_length_parameters[row];
                const std::size_t span = seed.find_span(s);
                const auto values =
                    seed.basis_function_derivatives(span, s, 0).front();
                for (std::size_t local = 0; local <= degree; ++local) {
                    const std::size_t expanded_index = span - degree + local;
                    matrix[row][expanded_index % unique_count] += values[local];
                }
            }

            std::vector<REAL> x_values(unique_count);
            std::vector<REAL> y_values(unique_count);
            for (std::size_t index = 0; index < unique_count; ++index) {
                x_values[index] = samples[index].x();
                y_values[index] = samples[index].y();
            }
            for (std::size_t row = 0; row < unique_count; ++row) {
                const REAL s = arc_length_parameters[row];
                const std::size_t span = seed.find_span(s);
                const auto values =
                    seed.basis_function_derivatives(span, s, 0).front();
                for (std::size_t local = 0; local <= degree; ++local) {
                    const std::size_t expanded_index = span - degree + local;
                    if (expanded_index >= unique_count) {
                        x_values[row] -= values[local] * displacement.x();
                        y_values[row] -= values[local] * displacement.y();
                    }
                }
            }
            const auto x_controls =
                solve_linear_system(matrix, x_values, tolerance);
            const auto y_controls =
                solve_linear_system(matrix, y_values, tolerance);

            std::vector<point2<REAL>> control_points(control_count);
            for (std::size_t index = 0; index < unique_count; ++index) {
                control_points[index] = {x_controls[index], y_controls[index]};
            }
            for (std::size_t index = 0; index < degree; ++index) {
                control_points[unique_count + index] =
                    control_points[index] + displacement;
            }
            return nurbs_spline2(
                std::move(control_points),
                std::move(unit_weights),
                std::move(knots),
                degree,
                closed,
                period_count,
                tolerance);
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

        for (std::size_t j = 1; j <= n - degree; ++j) {
            REAL sum = REAL(0);
            for (std::size_t index = j; index < j + degree; ++index) {
                sum += arc_length_parameters[index];
            }
            knots[j + degree] = sum / static_cast<REAL>(degree);
        }

        std::vector<REAL> unit_weights(point_count, REAL(1));
        const nurbs_spline2 seed(samples, unit_weights, knots, degree, tolerance);
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
        for (std::size_t index = 0; index < point_count; ++index) {
            x_values[index] = samples[index].x();
            y_values[index] = samples[index].y();
        }
        const auto x_controls = solve_linear_system(matrix, x_values, tolerance);
        const auto y_controls = solve_linear_system(matrix, y_values, tolerance);

        std::vector<point2<REAL>> control_points(point_count);
        for (std::size_t index = 0; index < point_count; ++index) {
            control_points[index] = {x_controls[index], y_controls[index]};
        }
        return nurbs_spline2(
            std::move(control_points),
            std::move(unit_weights),
            std::move(knots),
            degree,
            closed,
            std::size_t(1),
            tolerance);
    }

    /**
     * @brief Replace this curve with an interpolant through measured samples.
     * @param samples Positions in the 2D world to interpolate exactly.
     * @param arc_length_parameters Strictly increasing station for every sample.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param tolerance Positive solve and spline-definition tolerance.
     * @throws std::invalid_argument When sample data or parameters are invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    void adopt_to_points(
        const std::vector<point2<REAL>>& samples,
        const std::vector<REAL>& arc_length_parameters,
        std::size_t requested_degree = 3,
        REAL tolerance = REAL(1e-10)) {
        *this = interpolate(
            samples, arc_length_parameters, requested_degree, tolerance);
    }

    /**
     * @brief Replace this curve with a closed or repeated interpolant.
     * @param samples Positions in one fundamental period, including both ends.
     * @param arc_length_parameters Strictly increasing station for every sample.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param closed True to require coincident active-domain endpoints.
     * @param period_count Number of connected periods; zero means unlimited.
     * @param tolerance Positive solve, seam, and definition tolerance.
     * @throws std::invalid_argument When sample data or repeated input is invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    void adopt_to_points(
        const std::vector<point2<REAL>>& samples,
        const std::vector<REAL>& arc_length_parameters,
        std::size_t requested_degree,
        bool closed,
        std::size_t period_count,
        REAL tolerance = REAL(1e-10)) {
        *this = interpolate(
            samples,
            arc_length_parameters,
            requested_degree,
            closed,
            period_count,
            tolerance);
    }

private:
    [[nodiscard]] REAL period_s_min() const noexcept { return knots_[degree_]; }

    void validate_definition() const {
        if (!(tolerance_ > REAL(0)) || !std::isfinite(tolerance_)) {
            throw std::invalid_argument("2D NURBS tolerance must be finite and positive");
        }
        if (degree_ == 0) {
            throw std::invalid_argument("2D NURBS degree must be positive");
        }
        if (control_points_.size() <= degree_) {
            throw std::invalid_argument(
                "2D NURBS requires more control points than its degree");
        }
        if (weights_.size() != control_points_.size()) {
            throw std::invalid_argument(
                "2D NURBS weights and control points must have equal size");
        }
        if (knots_.size() != control_points_.size() + degree_ + 1) {
            throw std::invalid_argument(
                "2D NURBS knot count must equal control count + degree + 1");
        }
        for (const point2<REAL>& point : control_points_) {
            if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
                throw std::invalid_argument("2D NURBS control points must be finite");
            }
        }
        for (REAL weight : weights_) {
            if (!(weight > REAL(0)) || !std::isfinite(weight)) {
                throw std::invalid_argument("2D NURBS weights must be finite and positive");
            }
        }
        for (std::size_t index = 0; index < knots_.size(); ++index) {
            if (!std::isfinite(knots_[index])) {
                throw std::invalid_argument("2D NURBS knots must be finite");
            }
            if (index > 0 && knots_[index] < knots_[index - 1]) {
                throw std::invalid_argument("2D NURBS knots must be nondecreasing");
            }
        }
        if (!(period_s_max() > period_s_min())) {
            throw std::invalid_argument(
                "2D NURBS active parameter domain must have positive length");
        }
    }

    [[nodiscard]] REAL cyclic_tolerance() const noexcept {
        REAL scale = REAL(1);
        for (const point2<REAL>& point : control_points_) {
            scale = std::max({scale, std::abs(point.x()), std::abs(point.y())});
        }
        return REAL(16) * tolerance_ * scale;
    }

    [[nodiscard]] REAL cyclic_parameter_tolerance() const noexcept {
        return REAL(16) * tolerance_ * std::max({
            REAL(1), std::abs(period_s_min()), std::abs(period_s_max())});
    }

    void validate_repeated_definition() const {
        if (period_count_ == 1) {
            return;
        }
        const std::size_t unique_count = control_points_.size() - degree_;
        if (unique_count <= degree_) {
            throw std::invalid_argument(
                "periodic 2D NURBS requires more unique controls than its degree");
        }
        const std::size_t end_index = control_points_.size();
        if (!(knots_[degree_ - 1] < knots_[degree_] &&
              knots_[degree_] < knots_[degree_ + 1] &&
              knots_[end_index - 1] < knots_[end_index] &&
              knots_[end_index] < knots_[end_index + 1])) {
            throw std::invalid_argument(
                "periodic 2D NURBS requires simple start and end knots");
        }
        const REAL seam_tolerance = cyclic_tolerance();
        REAL weight_scale = REAL(1);
        for (REAL weight : weights_) {
            weight_scale = std::max(weight_scale, std::abs(weight));
        }
        const REAL weight_tolerance = REAL(16) * tolerance_ * weight_scale;
        const vector2<REAL> control_displacement =
            control_points_[unique_count] - control_points_.front();
        for (std::size_t index = 0; index < degree_; ++index) {
            const vector2<REAL> current_displacement =
                control_points_[unique_count + index] - control_points_[index];
            if ((current_displacement - control_displacement).length() >
                    seam_tolerance ||
                std::abs(weights_[index] - weights_[unique_count + index]) >
                    weight_tolerance) {
                throw std::invalid_argument(
                    "periodic 2D NURBS must translate its first degree controls consistently and repeat their weights");
            }
        }

        const REAL period = period_length();
        const REAL parameter_tolerance = cyclic_parameter_tolerance();
        for (std::size_t index = 0; index <= 2 * degree_; ++index) {
            const REAL shifted = knots_[index + unique_count] - knots_[index];
            if (std::abs(shifted - period) > parameter_tolerance) {
                throw std::invalid_argument(
                    "periodic 2D NURBS knots must repeat after one period");
            }
        }
    }

    [[nodiscard]] parameter_mapping checked_parameter(REAL s) const {
        if (!std::isfinite(s)) {
            throw std::out_of_range("2D NURBS parameter s must be finite");
        }
        if (s < period_s_min() - tolerance_) {
            throw std::out_of_range(
                "2D NURBS parameter s is before the active path domain");
        }
        if (period_count_ != 0 && s > s_max() + tolerance_) {
            throw std::out_of_range(
                "2D NURBS parameter s is outside the active knot domain");
        }

        const REAL clamped_s = std::max(s, period_s_min());
        if (period_count_ != 0 && clamped_s >= s_max()) {
            return {
                period_s_max(),
                static_cast<REAL>(period_count_ - 1)};
        }
        if (period_count_ == 1) {
            return {std::clamp(clamped_s, period_s_min(), period_s_max()), REAL(0)};
        }

        const REAL offset = clamped_s - period_s_min();
        const REAL index = std::floor(offset / period_length());
        REAL local_offset = std::fmod(offset, period_length());
        if (local_offset < REAL(0)) {
            local_offset += period_length();
        }
        return {
            std::clamp(
                period_s_min() + local_offset,
                period_s_min(),
                period_s_max()),
            index};
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
                const std::ptrdiff_t rk = static_cast<std::ptrdiff_t>(r) -
                                          static_cast<std::ptrdiff_t>(k);
                const std::ptrdiff_t pk = static_cast<std::ptrdiff_t>(p) -
                                          static_cast<std::ptrdiff_t>(k);

                if (r >= k) {
                    work[destination][0] = work[source][0] /
                        ndu[static_cast<std::size_t>(pk + 1)]
                           [static_cast<std::size_t>(rk)];
                    derivative = work[destination][0] *
                        ndu[static_cast<std::size_t>(rk)]
                           [static_cast<std::size_t>(pk)];
                }

                const std::ptrdiff_t j1 = rk >= -1 ? 1 : -rk;
                const std::ptrdiff_t j2 =
                    static_cast<std::ptrdiff_t>(r) - 1 <= pk
                        ? static_cast<std::ptrdiff_t>(k) - 1
                        : static_cast<std::ptrdiff_t>(p) -
                              static_cast<std::ptrdiff_t>(r);
                for (std::ptrdiff_t j = j1; j <= j2; ++j) {
                    work[destination][static_cast<std::size_t>(j)] =
                        (work[source][static_cast<std::size_t>(j)] -
                         work[source][static_cast<std::size_t>(j - 1)]) /
                        ndu[static_cast<std::size_t>(pk + 1)]
                           [static_cast<std::size_t>(rk + j)];
                    derivative +=
                        work[destination][static_cast<std::size_t>(j)] *
                        ndu[static_cast<std::size_t>(rk + j)]
                           [static_cast<std::size_t>(pk)];
                }

                if (static_cast<std::ptrdiff_t>(r) <= pk) {
                    work[destination][k] = -work[source][k - 1] /
                        ndu[static_cast<std::size_t>(pk + 1)][r];
                    derivative += work[destination][k] * ndu[r][static_cast<std::size_t>(pk)];
                }
                derivatives[k][r] = derivative;
                std::swap(source, destination);
            }
        }

        REAL factor = static_cast<REAL>(p);
        for (std::size_t k = 1; k <= order; ++k) {
            for (std::size_t j = 0; j <= p; ++j) {
                derivatives[k][j] *= factor;
            }
            factor *= static_cast<REAL>(p - k);
        }
        return derivatives;
    }

    std::vector<point2<REAL>> control_points_;
    std::vector<REAL> weights_;
    std::vector<REAL> knots_;
    std::size_t degree_;
    REAL tolerance_;
    bool closed_ = false;
    std::size_t period_count_ = 1;
    vector2<REAL> period_displacement_{};
    point2<REAL> start_{};
    point2<REAL> end_{};
};

} // namespace nurbspath
