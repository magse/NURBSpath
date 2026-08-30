#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/point3.hpp"
#include "nurbspath/serialization.hpp"
#include "nurbspath/spline3_definition.hpp"
#include "nurbspath/utility.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <istream>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
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
 * Checked definition setters retain the degree, validate a complete candidate
 * before committing it, and refresh cached endpoint state. A failed setter
 * leaves the spline unchanged. A successful setter invalidates references,
 * pointers, and iterators previously obtained from definition getters.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class nurbs_spline3 {
public:
    /**
     * @brief Construct a 3D NURBS curve from a detached definition value.
     *
     * The definition fields are moved from an rvalue or copied from an
     * lvalue. The degree remains separate from `spline3_definition` and is
     * validated together with the complete candidate.
     *
     * @param definition Owning control-point, weight, knot, closure, and
     * tolerance definition.
     * @param degree Positive degree below the control-point count.
     * @throws std::invalid_argument When the definition, degree, or closed
     * seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    nurbs_spline3(spline3_definition<REAL> definition, std::size_t degree)
        : nurbs_spline3(
              std::move(definition.control_points),
              std::move(definition.weights),
              std::move(definition.knots),
              degree,
              definition.closed,
              definition.tolerance) {}

    /**
     * @brief Construct a NURBS curve from its complete definition.
     * @param control_points World-space control points.
     * @param weights Positive rational weight for every control point.
     * @param knots Nondecreasing vector with
     * `nurbs_knot_count(degree, control_points.size())` values.
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
     * @param knots Nondecreasing vector with
     * `nurbs_knot_count(degree, control_points.size())` values.
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
        validate_and_refresh();
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
     * @brief Clone every editable definition field into a detached value.
     *
     * Editing the returned snapshot does not affect this spline until it is
     * supplied to `set_definition`. The polynomial degree is deliberately not
     * part of the returned aggregate.
     *
     * @return Deep-copy snapshot of control points, weights, knots, closure,
     * and tolerance.
     * @throws std::bad_alloc When allocating the copied vectors fails.
     */
    [[nodiscard]] spline3_definition<REAL> definition() const {
        return {
            .control_points = control_points_,
            .weights = weights_,
            .knots = knots_,
            .closed = closed_,
            .tolerance = tolerance_};
    }

    /**
     * @brief Get the number of editable scalar parameters.
     *
     * Control-point coordinates appear first in point-major X/Y/Z order,
     * followed by all weights and all knots. Degree, closure, and tolerance
     * are excluded.
     *
     * @return Current number of indexed scalar parameters.
     */
    [[nodiscard]] std::size_t number_of_parameters() const noexcept {
        return control_points_.size() * std::size_t(3) +
               weights_.size() + knots_.size();
    }

    /**
     * @brief Get one editable scalar parameter by its flattened index.
     * @param index Zero-based scalar parameter index.
     * @return Current parameter value.
     * @throws std::out_of_range When index is not below
     * `number_of_parameters()`.
     */
    [[nodiscard]] REAL get_parameter(std::size_t index) const {
        const std::size_t coordinate_count =
            control_points_.size() * std::size_t(3);
        if (index < coordinate_count) {
            const point3<REAL>& point = control_points_[index / 3];
            switch (index % 3) {
            case 0:
                return point.x;
            case 1:
                return point.y;
            default:
                return point.z;
            }
        }
        index -= coordinate_count;

        if (index < weights_.size()) {
            return weights_[index];
        }
        index -= weights_.size();

        if (index < knots_.size()) {
            return knots_[index];
        }
        throw std::out_of_range("3D NURBS parameter index is out of range");
    }

    /**
     * @brief Get the descriptive name of one scalar parameter in the current
     * definition.
     *
     * Names use `P[i].x`, `P[i].y`, `P[i].z`, `W[i]`, and `K[i]`, with
     * zero-based vector indices.
     *
     * @param index Zero-based scalar parameter index.
     * @return Parameter name in the documented flattened order.
     * @throws std::out_of_range When index is not below
     * `number_of_parameters()`.
     * @throws std::bad_alloc When allocating the returned string fails.
     */
    [[nodiscard]] std::string parameter_name(std::size_t index) const {
        const std::size_t coordinate_count =
            control_points_.size() * std::size_t(3);
        if (index < coordinate_count) {
            const std::size_t point_index = index / 3;
            constexpr char coordinates[] = {'x', 'y', 'z'};
            return "P[" + std::to_string(point_index) + "]." +
                   coordinates[index % 3];
        }
        index -= coordinate_count;

        if (index < weights_.size()) {
            return "W[" + std::to_string(index) + "]";
        }
        index -= weights_.size();

        if (index < knots_.size()) {
            return "K[" + std::to_string(index) + "]";
        }
        throw std::out_of_range("3D NURBS parameter index is out of range");
    }

    /**
     * @brief Validate and atomically replace one scalar parameter.
     *
     * A complete detached candidate is built and validated immediately.
     * Degree, closure, and tolerance remain unchanged. Returning false leaves
     * the spline, its active domain, and cached endpoints unchanged. Correlated
     * edits that cannot be valid one at a time should use `set_definition`
     * instead.
     *
     * @param index Zero-based scalar parameter index.
     * @param value Candidate scalar value.
     * @return True when the complete candidate is valid and committed; false
     * for an out-of-range index or rejected spline definition.
     * @throws std::bad_alloc When allocating the detached candidate fails.
     */
    [[nodiscard]] bool set_parameter(std::size_t index, REAL value) {
        if (index >= number_of_parameters()) {
            return false;
        }

        spline3_definition<REAL> candidate = definition();
        const std::size_t coordinate_count =
            candidate.control_points.size() * std::size_t(3);
        if (index < coordinate_count) {
            point3<REAL>& point = candidate.control_points[index / 3];
            switch (index % 3) {
            case 0:
                point.x = value;
                break;
            case 1:
                point.y = value;
                break;
            default:
                point.z = value;
                break;
            }
        } else {
            index -= coordinate_count;
            if (index < candidate.weights.size()) {
                candidate.weights[index] = value;
            } else {
                index -= candidate.weights.size();
                candidate.knots[index] = value;
            }
        }

        try {
            set_definition(std::move(candidate));
        } catch (const std::invalid_argument&) {
            return false;
        } catch (const std::domain_error&) {
            return false;
        }
        return true;
    }

    /**
     * @brief Get one control point by index.
     * @param index Zero-based control-point index.
     * @return Constant reference to the selected control point.
     * @throws std::out_of_range When index is outside the control-point vector.
     */
    [[nodiscard]] const point3<REAL>& control_point(std::size_t index) const {
        return control_points_.at(index);
    }

    /**
     * @brief Get one rational weight by index.
     * @param index Zero-based weight index.
     * @return Selected rational weight.
     * @throws std::out_of_range When index is outside the weight vector.
     */
    [[nodiscard]] REAL weight(std::size_t index) const {
        return weights_.at(index);
    }

    /**
     * @brief Get one knot by index.
     * @param index Zero-based knot index.
     * @return Selected knot value.
     * @throws std::out_of_range When index is outside the knot vector.
     */
    [[nodiscard]] REAL knot(std::size_t index) const {
        return knots_.at(index);
    }

    /**
     * @brief Replace one control point through a checked atomic update.
     * @param index Zero-based control-point index.
     * @param point New finite world-space control point.
     * @throws std::out_of_range When index is outside the control-point vector.
     * @throws std::invalid_argument When the resulting definition or seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero homogeneous weight.
     */
    void set_control_point(std::size_t index, const point3<REAL>& point) {
        std::vector<point3<REAL>> updated = control_points_;
        updated.at(index) = point;
        set_control_points(std::move(updated));
    }

    /**
     * @brief Replace one rational weight through a checked atomic update.
     * @param index Zero-based weight index.
     * @param weight New finite positive weight.
     * @throws std::out_of_range When index is outside the weight vector.
     * @throws std::invalid_argument When the resulting definition or seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero homogeneous weight.
     */
    void set_weight(std::size_t index, REAL weight) {
        std::vector<REAL> updated = weights_;
        updated.at(index) = weight;
        set_weights(std::move(updated));
    }

    /**
     * @brief Replace one knot through a checked atomic update.
     * @param index Zero-based knot index.
     * @param knot New finite knot value.
     * @throws std::out_of_range When index is outside the knot vector.
     * @throws std::invalid_argument When knot order, domain, or seam becomes invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero homogeneous weight.
     */
    void set_knot(std::size_t index, REAL knot) {
        std::vector<REAL> updated = knots_;
        updated.at(index) = knot;
        set_knots(std::move(updated));
    }

    /**
     * @brief Replace the complete control-point vector while retaining other fields.
     *
     * The new count must remain compatible with the current weights, knots,
     * and degree. Use `set_definition` to change related counts together.
     *
     * @param control_points New finite world-space control points.
     * @throws std::invalid_argument When the resulting definition or seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero homogeneous weight.
     */
    void set_control_points(std::vector<point3<REAL>> control_points) {
        commit_definition(
            std::move(control_points), weights_, knots_, closed_, tolerance_);
    }

    /**
     * @brief Replace the complete weight vector while retaining other fields.
     * @param weights New finite positive weight for every control point.
     * @throws std::invalid_argument When the resulting definition or seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero homogeneous weight.
     */
    void set_weights(std::vector<REAL> weights) {
        commit_definition(
            control_points_, std::move(weights), knots_, closed_, tolerance_);
    }

    /**
     * @brief Replace the complete knot vector while retaining other fields.
     *
     * The native active domain may change. Supplying the whole vector allows
     * affine rescaling without invalid intermediate knot orderings.
     *
     * @param knots New finite nondecreasing knot vector.
     * @throws std::invalid_argument When the resulting definition or seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero homogeneous weight.
     */
    void set_knots(std::vector<REAL> knots) {
        commit_definition(
            control_points_, weights_, std::move(knots), closed_, tolerance_);
    }

    /**
     * @brief Set standard open-clamped knots on the native domain `[0, s_end]`.
     *
     * Each endpoint receives multiplicity `degree() + 1`. Any knots between
     * those endpoint blocks are spaced uniformly. For a single-span spline,
     * the first half of the knot vector is zero and the second half is
     * `s_end`. Every generated knot span must have a finite reciprocal. The
     * complete candidate definition is validated before commit.
     *
     * @param s_end Finite positive end of the new native parameter domain.
     * @throws std::invalid_argument When `s_end` or the resulting definition
     * or closed seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_standard_knots(REAL s_end) {
        set_standard_knots(REAL(0), s_end);
    }

    /**
     * @brief Set standard open-clamped knots on `[s_start, s_end]`.
     *
     * Each endpoint receives multiplicity `degree() + 1`. When the spline
     * has more than one knot span, simple interior knots divide the requested
     * domain uniformly. For a single-span spline, the knot vector consists
     * of equally sized `s_start` and `s_end` blocks. The complete candidate
     * definition is validated before commit. The scalar representation must
     * provide distinct interior knots and a finite reciprocal for every
     * generated knot-span length.
     *
     * @param s_start Finite start of the new native parameter domain.
     * @param s_end Finite end of the new native parameter domain, strictly
     * greater than `s_start` and separated from it by a finite distance.
     * @throws std::invalid_argument When either bound or the resulting
     * definition or closed seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_standard_knots(REAL s_start, REAL s_end) {
        if (!std::isfinite(s_start) || !std::isfinite(s_end) ||
            !(s_end > s_start) || !std::isfinite(s_end - s_start)) {
            throw std::invalid_argument(
                "standard NURBS knot domain must have finite positive length");
        }

        const std::size_t span_count = control_points_.size() - degree_;
        std::vector<REAL> standard_knots(knots_.size(), s_start);
        const auto has_usable_span = [](REAL lower, REAL upper) {
            const REAL length = upper - lower;
            return length > REAL(0) && std::isfinite(length) &&
                   std::isfinite(REAL(1) / length);
        };
        REAL previous_knot = s_start;
        for (std::size_t span = 1; span < span_count; ++span) {
            const REAL fraction =
                static_cast<REAL>(span) / static_cast<REAL>(span_count);
            const REAL interior_knot =
                std::lerp(s_start, s_end, fraction);
            if (!(interior_knot < s_end) ||
                !has_usable_span(previous_knot, interior_knot)) {
                throw std::invalid_argument(
                    "standard NURBS knot spans must be numerically usable");
            }
            standard_knots[degree_ + span] = interior_knot;
            previous_knot = interior_knot;
        }
        if (!has_usable_span(previous_knot, s_end)) {
            throw std::invalid_argument(
                "standard NURBS knot spans must be numerically usable");
        }
        for (std::size_t index = control_points_.size();
             index < standard_knots.size();
             ++index) {
            standard_knots[index] = s_end;
        }
        set_knots(std::move(standard_knots));
    }

    /**
     * @brief Change the validation and parameter-boundary tolerance atomically.
     * @param tolerance New finite positive tolerance.
     * @throws std::invalid_argument When tolerance or the resulting seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero homogeneous weight.
     */
    void set_tolerance(REAL tolerance) {
        commit_definition(
            control_points_, weights_, knots_, closed_, tolerance);
    }

    /**
     * @brief Change the closed-seam requirement atomically.
     * @param closed True to require coincident active-domain endpoints.
     * @throws std::invalid_argument When enabling closure on an open seam.
     * @throws std::domain_error When endpoint evaluation has near-zero homogeneous weight.
     */
    void set_closed(bool closed) {
        commit_definition(
            control_points_, weights_, knots_, closed, tolerance_);
    }

    /**
     * @brief Atomically replace every definition field except the degree.
     *
     * The current degree is retained. The complete candidate is validated and
     * cached endpoints are refreshed before it replaces this spline.
     *
     * @param control_points New finite world-space control points.
     * @param weights New finite positive weight for every control point.
     * @param knots New finite nondecreasing knot vector.
     * @param closed True to require coincident active-domain endpoints.
     * @param tolerance New finite positive tolerance.
     * @throws std::invalid_argument When the resulting definition or seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero homogeneous weight.
     */
    void set_definition(
        std::vector<point3<REAL>> control_points,
        std::vector<REAL> weights,
        std::vector<REAL> knots,
        bool closed,
        REAL tolerance) {
        commit_definition(
            std::move(control_points),
            std::move(weights),
            std::move(knots),
            closed,
            tolerance);
    }

    /**
     * @brief Atomically adopt a detached definition while retaining degree.
     *
     * The complete candidate is validated and cached endpoints are refreshed
     * before commit. Failure leaves this spline and its degree unchanged.
     *
     * @param definition Owning candidate containing every editable field.
     * @throws std::invalid_argument When the candidate or closed seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero homogeneous weight.
     */
    void set_definition(spline3_definition<REAL> definition) {
        commit_definition(
            std::move(definition.control_points),
            std::move(definition.weights),
            std::move(definition.knots),
            definition.closed,
            definition.tolerance);
    }

    /**
     * @brief Write one version-1 tagged `spline3` text record.
     *
     * The single row stores the decimal tag, `spline3` and `v1` tokens,
     * degree, closure flag, tolerance, counts, interleaved control-point and
     * weight data, and the knot vector. Floating fields use classic-locale,
     * round-trip scientific notation. The row ends with one newline and does
     * not change caller formatting or locale. No explicit flush is requested;
     * normal stream policy applies. The complete grammar is documented in
     * `DATA.md`.
     *
     * @param tag Application-defined record tag; repeated tags are allowed.
     * @param output Destination text stream.
     * @return Reference to output after attempting to write one
     * newline-terminated row.
     * @throws std::bad_alloc When buffering the encoded row fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    std::ostream& tag_write(std::size_t tag, std::ostream& output) const {
        return detail::write_tagged_text_record<REAL>(
            output, tag, "spline3", [this](std::ostream& row) {
                row << ' ' << degree_ << ' ' << (closed_ ? 1 : 0) << ' '
                    << tolerance_ << ' ' << control_points_.size() << ' '
                    << knots_.size();
                for (std::size_t index = 0; index < control_points_.size();
                     ++index) {
                    const point3<REAL>& point = control_points_[index];
                    row << ' ' << point.x << ' ' << point.y << ' ' << point.z
                        << ' ' << weights_[index];
                }
                for (const REAL knot : knots_) {
                    row << ' ' << knot;
                }
            });
    }

    /**
     * @brief Read and allocate one version-1 tagged `spline3` text record.
     *
     * When a row is present, exactly that physical row is consumed. Its type
     * token must be `spline3`, and its complete definition is validated by the
     * spline constructor. Malformed data and type mismatches consume the row
     * and set `failbit`.
     *
     * @param input Source text stream positioned at the start of a row.
     * @return Tag and non-null shared spline after success, or `std::nullopt`
     * at EOF, another read failure, or after a malformed row.
     * @throws std::bad_alloc When buffering or allocating the spline definition fails.
     * @throws std::ios_base::failure When enabled by the stream exception mask.
     */
    [[nodiscard]] static std::optional<tagged_read_result<nurbs_spline3>>
    tag_read(std::istream& input);

    /**
     * @brief Get the cached point at the start of the active domain.
     *
     * The endpoint is derived from the spline definition and has no direct
     * setter. To target a different start point, update the control-point
     * definition through `set_control_point` or `set_control_points`.
     * With endpoint-clamped knots it equals the first control point; with
     * non-clamped knots it can depend on several controls and weights.
     *
     * @return Constant reference to the point at `s_min()` without evaluation.
     */
    [[nodiscard]] const point3<REAL>& get_start() const noexcept { return start_; }

    /**
     * @brief Get the cached point at the end of the active domain.
     *
     * The endpoint is derived from the spline definition and has no direct
     * setter. To target a different endpoint, update the control-point
     * definition through `set_control_point` or `set_control_points`.
     * With endpoint-clamped knots it equals the final control point; with
     * non-clamped knots it can depend on several controls and weights.
     *
     * @return Constant reference to the point at `s_max()` without evaluation.
     */
    [[nodiscard]] const point3<REAL>& get_end() const noexcept { return end_; }

    /** @brief Get the lower active parameter bound. @return Minimum valid s. */
    [[nodiscard]] REAL s_min() const noexcept { return knots_[degree_]; }
    /** @brief Get the upper active parameter bound. @return Maximum valid s. */
    [[nodiscard]] REAL s_max() const noexcept { return knots_[control_points_.size()]; }

    /**
     * @brief Evaluate position and the first two derivatives together.
     *
     * Repeated knots can create empty spans at an active-domain boundary.
     * At `s_min()` those spans are skipped toward the first nonempty span on
     * the right; at `s_max()` they are skipped toward the last nonempty span
     * on the left.
     *
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
        std::vector<REAL> knots(
            nurbs_knot_count(degree, point_count), REAL(0));

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
    void validate_and_refresh() {
        validate_definition();
        const spline_derivatives3<REAL> start_values = derivatives_at(s_min());
        const spline_derivatives3<REAL> end_values = derivatives_at(s_max());
        start_ = start_values.point;
        end_ = end_values.point;
        if (closed_ && distance(start_, end_) > closure_tolerance()) {
            throw std::invalid_argument("closed NURBS endpoints must coincide");
        }
    }

    void commit_definition(
        std::vector<point3<REAL>> control_points,
        std::vector<REAL> weights,
        std::vector<REAL> knots,
        bool closed,
        REAL tolerance) {
        nurbs_spline3 replacement(
            std::move(control_points),
            std::move(weights),
            std::move(knots),
            degree_,
            closed,
            tolerance);
        *this = std::move(replacement);
    }

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
        if (knots_.size() !=
            nurbs_knot_count(degree_, control_points_.size())) {
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
            // Endpoint evaluation is left-sided. Repeated boundary knots can
            // leave span n empty, so move to the final nonempty active span.
            std::size_t span = n;
            while (span > degree_ && knots_[span] == knots_[span + 1]) {
                --span;
            }
            return span;
        }
        if (s <= knots_[degree_]) {
            // Endpoint evaluation is right-sided. Repeated boundary knots can
            // leave span degree_ empty, so move to the first nonempty span.
            std::size_t span = degree_;
            while (span < n && knots_[span] == knots_[span + 1]) {
                ++span;
            }
            return span;
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

/**
 * @brief Definition-centric spelling of `nurbs_spline3`.
 *
 * This exact alias preserves interoperability with all APIs that accept an
 * ordinary 3D spline. It does not create a distinct runtime spline type.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
using nurbs_defined_spline3 = nurbs_spline3<REAL>;

namespace detail {

/** @cond */

template <std::floating_point REAL>
[[nodiscard]] inline std::optional<
    tagged_read_result<nurbs_spline3<REAL>>>
decode_tagged_spline3(
    const tagged_text_record& record,
    std::istream& source) {
    if (record.entity_type != "spline3") {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }

    auto payload = tagged_payload_input(record.payload);
    std::string degree_token;
    std::string closed_token;
    std::string tolerance_token;
    std::string control_count_token;
    std::string knot_count_token;
    if (!(payload >> degree_token >> closed_token >> tolerance_token >>
          control_count_token >> knot_count_token)) {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }

    std::size_t degree = 0;
    std::size_t control_count = 0;
    std::size_t knot_count = 0;
    REAL tolerance = REAL(0);
    if (!parse_size_token(degree_token, degree) ||
        !parse_size_token(control_count_token, control_count) ||
        !parse_size_token(knot_count_token, knot_count) ||
        !parse_real_token(tolerance_token, tolerance) ||
        (closed_token != "0" && closed_token != "1") ||
        !(tolerance > REAL(0)) ||
        control_count > record.payload.size() ||
        knot_count > record.payload.size()) {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }

    try {
        if (knot_count != nurbs_knot_count(degree, control_count)) {
            mark_tagged_read_failure(source);
            return std::nullopt;
        }
    } catch (const std::invalid_argument&) {
        mark_tagged_read_failure(source);
        return std::nullopt;
    } catch (const std::overflow_error&) {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }

    std::vector<point3<REAL>> control_points;
    std::vector<REAL> weights;
    std::vector<REAL> knots;

    for (std::size_t index = 0; index < control_count; ++index) {
        point3<REAL> point;
        REAL weight = REAL(0);
        if (!read_real_token(payload, point.x) ||
            !read_real_token(payload, point.y) ||
            !read_real_token(payload, point.z) ||
            !read_real_token(payload, weight)) {
            mark_tagged_read_failure(source);
            return std::nullopt;
        }
        control_points.push_back(point);
        weights.push_back(weight);
    }

    for (std::size_t index = 0; index < knot_count; ++index) {
        REAL knot = REAL(0);
        if (!read_real_token(payload, knot)) {
            mark_tagged_read_failure(source);
            return std::nullopt;
        }
        knots.push_back(knot);
    }

    if (!tagged_payload_exhausted(payload)) {
        mark_tagged_read_failure(source);
        return std::nullopt;
    }

    try {
        return tagged_read_result<nurbs_spline3<REAL>>{
            record.tag,
            std::make_shared<nurbs_spline3<REAL>>(
                std::move(control_points),
                std::move(weights),
                std::move(knots),
                degree,
                closed_token == "1",
                tolerance)};
    } catch (const std::invalid_argument&) {
        mark_tagged_read_failure(source);
    } catch (const std::domain_error&) {
        mark_tagged_read_failure(source);
    }
    return std::nullopt;
}

/** @endcond */

} // namespace detail

template <std::floating_point REAL>
std::optional<tagged_read_result<nurbs_spline3<REAL>>>
nurbs_spline3<REAL>::tag_read(std::istream& input) {
    const auto record = detail::read_tagged_text_record(input);
    if (!record) {
        return std::nullopt;
    }
    return detail::decode_tagged_spline3<REAL>(*record, input);
}

} // namespace nurbspath
