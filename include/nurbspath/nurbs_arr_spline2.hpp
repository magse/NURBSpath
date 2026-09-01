#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/nurbs_spline2.hpp"
#include "nurbspath/point2.hpp"
#include "nurbspath/utility.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
#include <valarray>
#include <vector>

namespace nurbspath {

/**
 * @brief Self-contained 2D NURBS curve backed by one public scalar valarray.
 *
 * The public `parameters` array stores point-major X/Y control coordinates,
 * followed by one weight per control point and then the complete knot vector.
 * For degree `p`, control-point count `C`, and array size `N`, the layout is
 * valid when `N = 4*C + p + 1` and `C > p`. The degree, closure flag, and
 * tolerance are typed metadata and are not stored in the scalar array.
 *
 * Direct writes and replacements of `parameters` deliberately bypass checked
 * setters. Every geometric operation therefore validates the current public
 * array before using it. Control points are reconstructed as `point2` values;
 * no object-representation aliasing is used. Endpoints and the active domain
 * are derived on demand and never cached.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class nurbs_arr_spline2 {
public:
    /** @brief Floating-point scalar stored by this spline. */
    using real_t = REAL;

    /**
     * @brief Allocation-free range of flattened indices into `parameters`.
     *
     * The bounds are captured when a range getter is called. Query a new
     * range after replacing or resizing the public `parameters` array.
     */
    using parameter_index_range =
        std::ranges::iota_view<std::size_t, std::size_t>;

    /**
     * @brief Flattened control coordinates, weights, and knots.
     *
     * This value is public so fitting and optimization code can copy, replace,
     * or apply valarray expressions to the complete numeric definition. Such
     * edits are checked the next time `validate` or a geometric operation is
     * called.
     */
    std::valarray<REAL> parameters;

    /**
     * @brief Construct a standard open 2D spline with explicit tolerance.
     *
     * Control points are distributed uniformly from `start` to `end` with
     * `lerp`, all rational weights are one, and standard open-clamped knots
     * span `[0, distance(start, end)]`.
     *
     * @param start First control point and active-domain endpoint.
     * @param end Final control point and active-domain endpoint.
     * @param control_point_count Number of uniformly spaced control points.
     * @param degree Positive degree below `control_point_count`.
     * @param tolerance Positive definition and parameter-boundary tolerance.
     * @throws std::invalid_argument When the points, counts, degree, distance,
     * tolerance, or generated definition is invalid.
     * @throws std::overflow_error When the flattened size is not representable.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    nurbs_arr_spline2(
        const point2<REAL>& start,
        const point2<REAL>& end,
        std::size_t control_point_count,
        std::size_t degree,
        REAL tolerance)
        : nurbs_arr_spline2(
              start,
              end,
              control_point_count,
              degree,
              false,
              tolerance) {}

    /**
     * @brief Construct a standard open or closed spline between two points.
     *
     * Control points are distributed uniformly from `start` to `end` with
     * `lerp`, all rational weights are one, and standard open-clamped knots
     * span `[0, distance(start, end)]`. A closed definition must also satisfy
     * the ordinary closed-seam validation.
     *
     * @tparam CLOSED Exact Boolean closure-flag type; defaults to `bool`.
     * @param start First control point and active-domain endpoint.
     * @param end Final control point and active-domain endpoint.
     * @param control_point_count Number of uniformly spaced control points.
     * @param degree Positive degree below `control_point_count`.
     * @param closed True when active-domain endpoints must coincide; defaults
     * to false.
     * @param tolerance Positive definition and parameter-boundary tolerance;
     * defaults to `64 * std::numeric_limits<REAL>::epsilon()`.
     * @throws std::invalid_argument When the points, counts, degree, distance,
     * tolerance, generated definition, or requested seam is invalid.
     * @throws std::overflow_error When the flattened size is not representable.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    template <std::same_as<bool> CLOSED = bool>
    nurbs_arr_spline2(
        const point2<REAL>& start,
        const point2<REAL>& end,
        std::size_t control_point_count,
        std::size_t degree,
        CLOSED closed = false,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : nurbs_arr_spline2(
              standard_parameters(start, end, control_point_count, degree),
              degree,
              false,
              tolerance) {
        set_standard_knots(REAL(0), distance(start, end));
        if (closed) {
            set_closed(true);
        }
    }

    /**
     * @brief Construct an open array-backed spline from flattened parameters.
     * @param values Point coordinates, weights, and knots in documented order.
     * @param degree Positive degree below the derived control-point count.
     * @param tolerance Positive definition and parameter-boundary tolerance.
     * @throws std::invalid_argument When the layout or definition is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    nurbs_arr_spline2(
        std::valarray<REAL> values,
        std::size_t degree,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : nurbs_arr_spline2(
              std::move(values), degree, false, tolerance) {}

    /**
     * @brief Construct an open or closed spline from flattened parameters.
     * @tparam CLOSED Exact Boolean closure-flag type.
     * @param values Point coordinates, weights, and knots in documented order.
     * @param degree Positive degree below the derived control-point count.
     * @param closed True when active-domain endpoints must coincide.
     * @param tolerance Positive validation and parameter-boundary tolerance.
     * @throws std::invalid_argument When the layout, definition, or seam is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    template <std::same_as<bool> CLOSED>
    nurbs_arr_spline2(
        std::valarray<REAL> values,
        std::size_t degree,
        CLOSED closed,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : parameters(std::move(values)),
          degree_(degree),
          tolerance_(tolerance),
          closed_(closed) {
        validate();
    }

    /**
     * @brief Construct an open array-backed spline from component vectors.
     * @param control_points Finite control points in the independent 2D world.
     * @param weights Finite positive rational weight for every control point.
     * @param knots Complete finite nondecreasing knot vector.
     * @param degree Positive degree below the control-point count.
     * @param tolerance Positive definition and parameter-boundary tolerance.
     * @throws std::invalid_argument When the definition is invalid.
     * @throws std::overflow_error When the flattened size is not representable.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    nurbs_arr_spline2(
        std::vector<point2<REAL>> control_points,
        std::vector<REAL> weights,
        std::vector<REAL> knots,
        std::size_t degree,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : nurbs_arr_spline2(
              flatten_components(control_points, weights, knots, degree),
              degree,
              false,
              tolerance) {}

    /**
     * @brief Construct an open or closed spline from component vectors.
     * @tparam CLOSED Exact Boolean closure-flag type.
     * @param control_points Finite control points in the independent 2D world.
     * @param weights Finite positive rational weight for every control point.
     * @param knots Complete finite nondecreasing knot vector.
     * @param degree Positive degree below the control-point count.
     * @param closed True when active-domain endpoints must coincide.
     * @param tolerance Positive validation and parameter-boundary tolerance.
     * @throws std::invalid_argument When the definition or seam is invalid.
     * @throws std::overflow_error When the flattened size is not representable.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    template <std::same_as<bool> CLOSED>
    nurbs_arr_spline2(
        std::vector<point2<REAL>> control_points,
        std::vector<REAL> weights,
        std::vector<REAL> knots,
        std::size_t degree,
        CLOSED closed,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : nurbs_arr_spline2(
              flatten_components(control_points, weights, knots, degree),
              degree,
              closed,
              tolerance) {}

    /**
     * @brief Copy an ordinary 2D NURBS spline into flattened storage.
     * @param spline Valid ordinary spline to copy.
     * @throws std::overflow_error When the flattened size is not representable.
     * @throws std::bad_alloc When allocating the copied valarray fails.
     */
    explicit nurbs_arr_spline2(const nurbs_spline2<REAL>& spline)
        : parameters(flatten_components(
              spline.get_control_points(),
              spline.get_weights(),
              spline.get_knots(),
              spline.degree())),
          degree_(spline.degree()),
          tolerance_(spline.tolerance()),
          closed_(spline.is_closed()) {}

    /**
     * @brief Get the polynomial degree.
     * @return Positive spline degree.
     */
    [[nodiscard]] std::size_t degree() const noexcept { return degree_; }

    /**
     * @brief Get the definition and parameter-boundary tolerance.
     * @return Current tolerance.
     */
    [[nodiscard]] REAL tolerance() const noexcept { return tolerance_; }

    /**
     * @brief Report whether the spline requires a closed seam.
     * @return True when active-domain endpoints must coincide.
     */
    [[nodiscard]] bool is_closed() const noexcept { return closed_; }

    /**
     * @brief Get the number of flattened scalar parameters.
     * @return Current public array size, including malformed layouts.
     */
    [[nodiscard]] std::size_t number_of_parameters() const noexcept {
        return parameters.size();
    }

    /**
     * @brief Derive the control-point count from the current array layout.
     * @return Number of represented control points.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] std::size_t control_point_count() const {
        return checked_layout().control_count;
    }

    /**
     * @brief Derive the knot count from the current array layout.
     * @return Number of represented knots.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] std::size_t knot_count() const {
        return checked_layout().knot_count;
    }

    /**
     * @brief Derive the weight count from the current array layout.
     * @return Number of represented weights, equal to the control-point count.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] std::size_t weight_count() const {
        return checked_layout().control_count;
    }

    /**
     * @brief Get the flattened indices occupied by control-point coordinates.
     *
     * The returned half-open range counts individual X/Y scalar values, so it
     * contains twice as many indices as there are control points. Its values
     * can be used directly to index the public `parameters` array.
     *
     * @return Iterable scalar-index range `[0, 2 * control_point_count())`.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] parameter_index_range get_control_point_count_range() const {
        const layout_info layout = checked_layout();
        return std::views::iota(std::size_t(0), layout.weight_offset);
    }

    /**
     * @brief Get the flattened indices occupied by rational weights.
     *
     * The returned half-open range contains one scalar index per weight. Its
     * values can be used directly to index the public `parameters` array.
     *
     * @return Iterable scalar-index range `[2 * C, 3 * C)`, where `C` is the
     * control-point count.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] parameter_index_range get_weight_count_range() const {
        const layout_info layout = checked_layout();
        return std::views::iota(layout.weight_offset, layout.knot_offset);
    }

    /**
     * @brief Get the flattened indices occupied by knots.
     *
     * The returned half-open range contains one scalar index per knot. Its
     * values can be used directly to index the public `parameters` array.
     *
     * @return Iterable scalar-index range `[3 * C, parameters.size())`, where
     * `C` is the control-point count.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] parameter_index_range get_knot_count_range() const {
        const layout_info layout = checked_layout();
        return std::views::iota(layout.knot_offset, parameters.size());
    }

    /**
     * @brief Get all control points as detached point values.
     * @return Control points reconstructed in index order.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::bad_alloc When allocating the result fails.
     */
    [[nodiscard]] std::vector<point2<REAL>> get_control_points() const {
        const layout_info layout = checked_layout();
        return control_points_for(layout);
    }

    /**
     * @brief Get all weights as a detached vector.
     * @return Weights copied in index order.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::bad_alloc When allocating the result fails.
     */
    [[nodiscard]] std::vector<REAL> get_weights() const {
        const layout_info layout = checked_layout();
        return weights_for(layout);
    }

    /**
     * @brief Get all knots as a detached vector.
     * @return Knots copied in index order.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::bad_alloc When allocating the result fails.
     */
    [[nodiscard]] std::vector<REAL> get_knots() const {
        const layout_info layout = checked_layout();
        return knots_for(layout);
    }

    /**
     * @brief Get one control point by index.
     * @param index Zero-based control-point index.
     * @return Point reconstructed by value from two adjacent scalars.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::out_of_range When index is outside the control-point range.
     */
    [[nodiscard]] point2<REAL> get_control_point(std::size_t index) const {
        const layout_info layout = checked_layout();
        if (index >= layout.control_count) {
            throw std::out_of_range(
                "array-backed 2D NURBS control-point index is out of range");
        }
        return control_point_unchecked(index);
    }

    /**
     * @brief Get one rational weight by index.
     * @param index Zero-based weight index.
     * @return Selected weight value.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::out_of_range When index is outside the weight range.
     */
    [[nodiscard]] REAL get_weight(std::size_t index) const {
        const layout_info layout = checked_layout();
        if (index >= layout.control_count) {
            throw std::out_of_range(
                "array-backed 2D NURBS weight index is out of range");
        }
        return parameters[layout.weight_offset + index];
    }

    /**
     * @brief Get one knot by index.
     * @param index Zero-based knot index.
     * @return Selected knot value.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::out_of_range When index is outside the knot range.
     */
    [[nodiscard]] REAL get_knot(std::size_t index) const {
        const layout_info layout = checked_layout();
        if (index >= layout.knot_count) {
            throw std::out_of_range(
                "array-backed 2D NURBS knot index is out of range");
        }
        return parameters[layout.knot_offset + index];
    }

    /**
     * @brief Get one scalar by its flattened index.
     * @param index Zero-based scalar index in `parameters`.
     * @return Selected scalar value.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::out_of_range When index is outside the public array.
     */
    [[nodiscard]] REAL get_parameter(std::size_t index) const {
        static_cast<void>(checked_layout());
        if (index >= parameters.size()) {
            throw std::out_of_range(
                "array-backed 2D NURBS parameter index is out of range");
        }
        return parameters[index];
    }

    /**
     * @brief Get the descriptive name of one flattened scalar.
     * @param index Zero-based scalar index in `parameters`.
     * @return A `P[i].x`, `P[i].y`, `W[i]`, or `K[i]` name.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::out_of_range When index is outside the public array.
     * @throws std::bad_alloc When allocating the returned string fails.
     */
    [[nodiscard]] std::string parameter_name(std::size_t index) const {
        const layout_info layout = checked_layout();
        if (index >= parameters.size()) {
            throw std::out_of_range(
                "array-backed 2D NURBS parameter index is out of range");
        }
        const std::size_t coordinate_count = layout.weight_offset;
        if (index < coordinate_count) {
            const std::size_t point_index = index / 2;
            const char coordinate = index % 2 == 0 ? 'x' : 'y';
            return "P[" + std::to_string(point_index) + "]." + coordinate;
        }
        if (index < layout.knot_offset) {
            return "W[" + std::to_string(index - coordinate_count) + "]";
        }
        return "K[" + std::to_string(index - layout.knot_offset) + "]";
    }

    /**
     * @brief Validate the current public array and all typed metadata.
     * @throws std::invalid_argument When layout, values, domain, or seam is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void validate() const { static_cast<void>(validated_definition()); }

    /**
     * @brief Atomically validate and replace the complete public scalar array.
     * @param values Candidate flattened coordinates, weights, and knots.
     * @throws std::invalid_argument When the candidate definition is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_parameters(std::valarray<REAL> values) {
        nurbs_arr_spline2 replacement(
            std::move(values), degree_, closed_, tolerance_);
        *this = std::move(replacement);
    }

    /**
     * @brief Atomically copy and replace the complete public scalar array.
     *
     * Exactly `number_of_parameters()` consecutive values are copied before
     * validation, so `values` may point to the first element of this object's
     * current `parameters` storage. This overload cannot verify the source
     * buffer length; passing a non-null pointer to fewer readable values has
     * undefined behavior.
     *
     * @param values Pointer to at least `number_of_parameters()` flattened
     * coordinates, weights, and knots.
     * @throws std::invalid_argument When `values` is null or the copied
     * candidate definition is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     * @throws std::bad_alloc When allocating the detached candidate fails.
     */
    void set_parameters(const REAL* values) {
        if (values == nullptr) {
            throw std::invalid_argument(
                "array-backed 2D NURBS parameter pointer must not be null");
        }

        // Detach before validation and assignment because values may alias
        // the current public valarray storage.
        std::valarray<REAL> candidate(values, parameters.size());
        set_parameters(std::move(candidate));
    }

    /**
     * @brief Atomically validate and replace one flattened scalar.
     * @param index Zero-based scalar index in `parameters`.
     * @param value Candidate scalar value.
     * @return True when the candidate was valid and committed; false for an
     * out-of-range index, malformed current shape, or a rejected value.
     * @throws std::bad_alloc When allocating a detached candidate fails.
     */
    [[nodiscard]] bool set_parameter(std::size_t index, REAL value) {
        try {
            static_cast<void>(checked_layout());
            if (index >= parameters.size()) {
                return false;
            }
            std::valarray<REAL> candidate = parameters;
            candidate[index] = value;
            set_parameters(std::move(candidate));
        } catch (const std::invalid_argument&) {
            return false;
        } catch (const std::domain_error&) {
            return false;
        }
        return true;
    }

    /**
     * @brief Atomically replace one control point.
     * @param index Zero-based control-point index.
     * @param point Candidate finite point.
     * @throws std::invalid_argument When the current shape or candidate is invalid.
     * @throws std::out_of_range When index is outside the control-point range.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_control_point(std::size_t index, const point2<REAL>& point) {
        const layout_info layout = checked_layout();
        if (index >= layout.control_count) {
            throw std::out_of_range(
                "array-backed 2D NURBS control-point index is out of range");
        }
        std::valarray<REAL> candidate = parameters;
        candidate[2 * index] = point.x;
        candidate[2 * index + 1] = point.y;
        set_parameters(std::move(candidate));
    }

    /**
     * @brief Atomically replace one rational weight.
     * @param index Zero-based weight index.
     * @param value Candidate finite positive weight.
     * @throws std::invalid_argument When the current shape or candidate is invalid.
     * @throws std::out_of_range When index is outside the weight range.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_weight(std::size_t index, REAL value) {
        const layout_info layout = checked_layout();
        if (index >= layout.control_count) {
            throw std::out_of_range(
                "array-backed 2D NURBS weight index is out of range");
        }
        std::valarray<REAL> candidate = parameters;
        candidate[layout.weight_offset + index] = value;
        set_parameters(std::move(candidate));
    }

    /**
     * @brief Atomically replace one knot.
     * @param index Zero-based knot index.
     * @param value Candidate finite knot value.
     * @throws std::invalid_argument When the current shape or candidate is invalid.
     * @throws std::out_of_range When index is outside the knot range.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_knot(std::size_t index, REAL value) {
        const layout_info layout = checked_layout();
        if (index >= layout.knot_count) {
            throw std::out_of_range(
                "array-backed 2D NURBS knot index is out of range");
        }
        std::valarray<REAL> candidate = parameters;
        candidate[layout.knot_offset + index] = value;
        set_parameters(std::move(candidate));
    }

    /**
     * @brief Atomically replace all control points while retaining other data.
     * @param control_points Candidate control points.
     * @throws std::invalid_argument When counts or resulting definition are invalid.
     * @throws std::overflow_error When the flattened size is not representable.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_control_points(std::vector<point2<REAL>> control_points) {
        set_definition(
            std::move(control_points),
            get_weights(),
            get_knots(),
            closed_,
            tolerance_);
    }

    /**
     * @brief Atomically replace all weights while retaining other data.
     * @param values Candidate finite positive weights.
     * @throws std::invalid_argument When counts or resulting definition are invalid.
     * @throws std::overflow_error When the flattened size is not representable.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_weights(std::vector<REAL> values) {
        set_definition(
            get_control_points(),
            std::move(values),
            get_knots(),
            closed_,
            tolerance_);
    }

    /**
     * @brief Atomically replace all knots while retaining other data.
     * @param values Candidate finite nondecreasing knot vector.
     * @throws std::invalid_argument When counts or resulting definition are invalid.
     * @throws std::overflow_error When the flattened size is not representable.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_knots(std::vector<REAL> values) {
        set_definition(
            get_control_points(),
            get_weights(),
            std::move(values),
            closed_,
            tolerance_);
    }

    /**
     * @brief Set standard open-clamped knots on `[0, s_end]`.
     * @param s_end Finite positive end of the new native parameter domain.
     * @throws std::invalid_argument When the requested domain or definition is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_standard_knots(REAL s_end) {
        set_standard_knots(REAL(0), s_end);
    }

    /**
     * @brief Set standard open-clamped knots on `[s_start, s_end]`.
     *
     * Endpoint multiplicity is `degree() + 1`; any interior simple knots are
     * uniformly spaced. Every generated span must have finite positive length
     * and a finite reciprocal in `REAL`.
     *
     * @param s_start Finite start of the new native parameter domain.
     * @param s_end Finite end strictly greater than `s_start`.
     * @throws std::invalid_argument When the requested domain or definition is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_standard_knots(REAL s_start, REAL s_end) {
        const validated_state state = validated_definition();
        if (!std::isfinite(s_start) || !std::isfinite(s_end) ||
            !(s_end > s_start) || !std::isfinite(s_end - s_start)) {
            throw std::invalid_argument(
                "standard array-backed 2D NURBS knot domain must have finite positive length");
        }

        const std::size_t span_count =
            state.layout.control_count - degree_;
        std::valarray<REAL> candidate = parameters;
        for (std::size_t index = 0; index < state.layout.knot_count; ++index) {
            candidate[state.layout.knot_offset + index] = s_start;
        }
        const auto has_usable_span = [](REAL lower, REAL upper) {
            const REAL length = upper - lower;
            return length > REAL(0) && std::isfinite(length) &&
                   std::isfinite(REAL(1) / length);
        };
        REAL previous_knot = s_start;
        for (std::size_t span = 1; span < span_count; ++span) {
            const REAL fraction =
                static_cast<REAL>(span) / static_cast<REAL>(span_count);
            const REAL interior_knot = std::lerp(s_start, s_end, fraction);
            if (!(interior_knot < s_end) ||
                !has_usable_span(previous_knot, interior_knot)) {
                throw std::invalid_argument(
                    "standard array-backed 2D NURBS knot spans must be numerically usable");
            }
            candidate[state.layout.knot_offset + degree_ + span] =
                interior_knot;
            previous_knot = interior_knot;
        }
        if (!has_usable_span(previous_knot, s_end)) {
            throw std::invalid_argument(
                "standard array-backed 2D NURBS knot spans must be numerically usable");
        }
        for (std::size_t index = state.layout.control_count;
             index < state.layout.knot_count;
             ++index) {
            candidate[state.layout.knot_offset + index] = s_end;
        }
        set_parameters(std::move(candidate));
    }

    /**
     * @brief Atomically replace the validation tolerance.
     * @param value Candidate finite positive tolerance.
     * @throws std::invalid_argument When the tolerance or resulting seam is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_tolerance(REAL value) {
        nurbs_arr_spline2 replacement(parameters, degree_, closed_, value);
        *this = std::move(replacement);
    }

    /**
     * @brief Atomically change the closed-seam requirement.
     * @param value True to require coincident active-domain endpoints.
     * @throws std::invalid_argument When the resulting seam is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_closed(bool value) {
        nurbs_arr_spline2 replacement(parameters, degree_, value, tolerance_);
        *this = std::move(replacement);
    }

    /**
     * @brief Atomically replace all numeric components and mutable metadata.
     * @param control_points Candidate finite 2D control points.
     * @param weights Candidate finite positive rational weights.
     * @param knots Candidate finite nondecreasing knot vector.
     * @param closed True to require coincident active-domain endpoints.
     * @param tolerance Candidate finite positive tolerance.
     * @throws std::invalid_argument When the resulting definition is invalid.
     * @throws std::overflow_error When the flattened size is not representable.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    void set_definition(
        std::vector<point2<REAL>> control_points,
        std::vector<REAL> weights,
        std::vector<REAL> knots,
        bool closed,
        REAL tolerance) {
        nurbs_arr_spline2 replacement(
            std::move(control_points),
            std::move(weights),
            std::move(knots),
            degree_,
            closed,
            tolerance);
        *this = std::move(replacement);
    }

    /**
     * @brief Get the lower active parameter bound from the current knot data.
     * @return Minimum valid native `s`.
     * @throws std::invalid_argument When the current definition is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    [[nodiscard]] REAL s_min() const {
        const validated_state state = validated_definition();
        return knot_unchecked(state.layout, degree_);
    }

    /**
     * @brief Get the upper active parameter bound from the current knot data.
     * @return Maximum valid native `s`.
     * @throws std::invalid_argument When the current definition is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    [[nodiscard]] REAL s_max() const {
        const validated_state state = validated_definition();
        return knot_unchecked(state.layout, state.layout.control_count);
    }

    /**
     * @brief Evaluate the current point at the active-domain start.
     * @return Start point derived on demand from public storage.
     * @throws std::invalid_argument When the current definition is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    [[nodiscard]] point2<REAL> get_start() const {
        return validated_definition().start;
    }

    /**
     * @brief Evaluate the current point at the active-domain end.
     * @return End point derived on demand from public storage.
     * @throws std::invalid_argument When the current definition is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     */
    [[nodiscard]] point2<REAL> get_end() const {
        return validated_definition().end;
    }

    /**
     * @brief Evaluate position and the first two analytic derivatives.
     * @param s Finite parameter in the active knot domain.
     * @return Position, first derivative, and second derivative in 2D.
     * @throws std::invalid_argument When the current definition is invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When the homogeneous weight is near zero.
     */
    [[nodiscard]] spline_derivatives2<REAL> derivatives_at(REAL s) const {
        const validated_state state = validated_definition();
        const REAL parameter = checked_parameter(s, state.layout);
        const auto derivatives =
            rational_derivatives_at(parameter, 2, state.layout);
        return {
            point2<REAL>{derivatives[0].x, derivatives[0].y},
            derivatives[1],
            derivatives[2]};
    }

    /**
     * @brief Evaluate the curve position.
     * @param s Finite parameter in the active knot domain.
     * @return Curve point in the independent 2D world.
     * @throws std::invalid_argument When the current definition is invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When the homogeneous weight is near zero.
     */
    [[nodiscard]] point2<REAL> evaluate(REAL s) const {
        return derivatives_at(s).point;
    }

    /**
     * @brief Evaluate the curve position.
     * @param s Finite parameter in the active knot domain.
     * @return Curve point in the independent 2D world.
     * @throws std::invalid_argument When the current definition is invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When the homogeneous weight is near zero.
     */
    [[nodiscard]] point2<REAL> point_at(REAL s) const { return evaluate(s); }

    /**
     * @brief Evaluate the first analytic derivative.
     * @param s Finite parameter in the active knot domain.
     * @return First derivative with respect to native `s`.
     * @throws std::invalid_argument When the current definition is invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When the homogeneous weight is near zero.
     */
    [[nodiscard]] vector2<REAL> first_derivative(REAL s) const {
        return derivatives_at(s).first;
    }

    /**
     * @brief Evaluate the second analytic derivative.
     * @param s Finite parameter in the active knot domain.
     * @return Second derivative with respect to native `s`.
     * @throws std::invalid_argument When the current definition is invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When the homogeneous weight is near zero.
     */
    [[nodiscard]] vector2<REAL> second_derivative(REAL s) const {
        return derivatives_at(s).second;
    }

    /**
     * @brief Evaluate the third analytic rational derivative.
     * @param s Finite parameter in the active knot domain.
     * @return Third derivative with respect to native `s`.
     * @throws std::invalid_argument When the current definition is invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When the homogeneous weight is near zero.
     */
    [[nodiscard]] vector2<REAL> third_derivative(REAL s) const {
        const validated_state state = validated_definition();
        const REAL parameter = checked_parameter(s, state.layout);
        return rational_derivatives_at(parameter, 3, state.layout)[3];
    }

    /**
     * @brief Evaluate a unit tangent.
     * @param s Finite parameter in the active knot domain.
     * @param tolerance Minimum accepted first-derivative length.
     * @return Normalized first derivative.
     * @throws std::invalid_argument When the definition is invalid.
     * @throws std::out_of_range When `s` is outside the domain.
     * @throws std::domain_error When the derivative is too small or a
     * homogeneous weight is near zero.
     */
    [[nodiscard]] vector2<REAL> tangent(
        REAL s,
        REAL tolerance = vector2<REAL>::default_tolerance()) const {
        return first_derivative(s).normalized(tolerance);
    }

    /**
     * @brief Evaluate unsigned geometric curvature in the 2D world.
     *
     * Curvature is calculated analytically as
     * `abs(C'(s) cross C''(s)) / length(C'(s))^3` and is independent of the
     * scale of the native parameter while the analytic derivatives remain
     * representable in `REAL`. At an internal knot without second-derivative
     * continuity, the result uses the right-hand nonempty span; `s_max()` uses
     * the left-hand nonempty span. This one-sided value does not represent
     * curvature at a corner.
     *
     * @param s Finite parameter in the active knot domain.
     * @param tangent_tolerance Nonnegative finite minimum accepted
     * first-derivative length.
     * @return Nonnegative curvature magnitude in inverse 2D world units.
     * @throws std::invalid_argument When the current definition is invalid or
     * `tangent_tolerance` is negative or non-finite.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When a homogeneous weight is near zero or the
     * first-derivative length is non-finite or not longer than
     * `tangent_tolerance`, or finite curvature cannot be represented.
     */
    [[nodiscard]] REAL curvature(
        REAL s,
        REAL tangent_tolerance = vector2<REAL>::default_tolerance()) const {
        if (!(tangent_tolerance >= REAL(0)) ||
            !std::isfinite(tangent_tolerance)) {
            throw std::invalid_argument(
                "curvature tangent tolerance must be finite and nonnegative");
        }

        const spline_derivatives2<REAL> derivatives = derivatives_at(s);
        const REAL speed = derivatives.first.length();
        if (!std::isfinite(speed) || !(speed > tangent_tolerance)) {
            throw std::domain_error(
                "curvature requires a finite nonzero 2D spline derivative");
        }
        const vector2<REAL> unit = derivatives.first / speed;
        const vector2<REAL> scaled_second = derivatives.second / speed;
        const REAL result = std::abs(unit.cross(scaled_second)) / speed;
        if (!std::isfinite(result)) {
            throw std::domain_error("cannot represent finite 2D spline curvature");
        }
        return result;
    }

    /**
     * @brief Estimate arc length using a uniform-native-s polyline.
     * @param segment_count Positive number of approximation segments.
     * @return Approximate length in 2D world units.
     * @throws std::invalid_argument When the definition is invalid or the
     * segment count is zero.
     * @throws std::domain_error When a homogeneous weight is near zero.
     */
    [[nodiscard]] REAL approximate_arc_length(
        std::size_t segment_count = 512) const {
        if (segment_count == 0) {
            throw std::invalid_argument("segment_count must be positive");
        }
        const validated_state state = validated_definition();
        const REAL lower = knot_unchecked(state.layout, degree_);
        const REAL upper =
            knot_unchecked(state.layout, state.layout.control_count);
        point2<REAL> previous = evaluate_validated(lower, state.layout);
        REAL length = REAL(0);
        for (std::size_t index = 0; index < segment_count; ++index) {
            const REAL fraction =
                static_cast<REAL>(index + 1) /
                static_cast<REAL>(segment_count);
            const REAL s = lower + fraction * (upper - lower);
            const point2<REAL> current = evaluate_validated(s, state.layout);
            length += distance(previous, current);
            previous = current;
        }
        return length;
    }

    /**
     * @brief Build an open unit-weight spline interpolating measured samples.
     * @param samples Positions in the independent 2D world.
     * @param arc_length_parameters Strictly increasing native stations.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param tolerance Positive solve and definition tolerance.
     * @return Array-backed interpolating curve.
     * @throws std::invalid_argument When samples, stations, or tolerance are invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    [[nodiscard]] static nurbs_arr_spline2 interpolate(
        const std::vector<point2<REAL>>& samples,
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
     * @brief Build an open or closed unit-weight interpolating spline.
     * @param samples Positions in the independent 2D world.
     * @param arc_length_parameters Strictly increasing native stations.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param closed True to require coincident active-domain endpoints.
     * @param tolerance Positive solve, seam, and definition tolerance.
     * @return Array-backed interpolating curve.
     * @throws std::invalid_argument When samples, stations, closure, or tolerance are invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    [[nodiscard]] static nurbs_arr_spline2 interpolate(
        const std::vector<point2<REAL>>& samples,
        const std::vector<REAL>& arc_length_parameters,
        std::size_t requested_degree,
        bool closed,
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
            throw std::invalid_argument(
                "interpolation tolerance must be finite and positive");
        }
        for (std::size_t index = 0;
             index < arc_length_parameters.size();
             ++index) {
            if (!std::isfinite(arc_length_parameters[index])) {
                throw std::invalid_argument(
                    "arc-length parameters must be finite");
            }
            if (index > 0 &&
                !(arc_length_parameters[index] >
                  arc_length_parameters[index - 1])) {
                throw std::invalid_argument(
                    "arc-length parameters must be strictly increasing");
            }
        }
        for (const point2<REAL>& sample : samples) {
            if (!std::isfinite(sample.x) || !std::isfinite(sample.y)) {
                throw std::invalid_argument(
                    "interpolation samples must be finite");
            }
        }
        if (closed &&
            distance(samples.front(), samples.back()) > tolerance) {
            throw std::invalid_argument(
                "closed array-backed 2D interpolation requires coincident first and final samples");
        }

        const std::size_t point_count = samples.size();
        const std::size_t degree =
            std::min(requested_degree, point_count - 1);
        const std::size_t n = point_count - 1;
        std::vector<REAL> knots(
            nurbs_knot_count(degree, point_count), REAL(0));
        std::fill_n(
            knots.begin(), degree + 1, arc_length_parameters.front());
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
        const nurbs_arr_spline2 seed(
            samples, unit_weights, knots, degree, tolerance);
        const validated_state seed_state = seed.validated_definition();
        std::vector<std::vector<REAL>> matrix(
            point_count, std::vector<REAL>(point_count, REAL(0)));
        for (std::size_t row = 0; row < point_count; ++row) {
            const REAL s = arc_length_parameters[row];
            const std::size_t span = seed.find_span(s, seed_state.layout);
            const auto values =
                seed.basis_function_derivatives(
                    span, s, 0, seed_state.layout).front();
            for (std::size_t local = 0; local <= degree; ++local) {
                matrix[row][span - degree + local] = values[local];
            }
        }

        std::vector<REAL> x_values(point_count);
        std::vector<REAL> y_values(point_count);
        for (std::size_t index = 0; index < point_count; ++index) {
            x_values[index] = samples[index].x;
            y_values[index] = samples[index].y;
        }
        const auto x_controls =
            solve_linear_system(matrix, x_values, tolerance);
        const auto y_controls =
            solve_linear_system(matrix, y_values, tolerance);
        std::vector<point2<REAL>> control_points(point_count);
        for (std::size_t index = 0; index < point_count; ++index) {
            control_points[index] = {x_controls[index], y_controls[index]};
        }
        return nurbs_arr_spline2(
            std::move(control_points),
            std::move(unit_weights),
            std::move(knots),
            degree,
            closed,
            tolerance);
    }

    /**
     * @brief Replace this curve with an open interpolant through samples.
     * @param samples Positions in the independent 2D world.
     * @param arc_length_parameters Strictly increasing native stations.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param tolerance Positive solve and definition tolerance.
     * @throws std::invalid_argument When samples or stations are invalid.
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
     * @brief Replace this curve with an open or closed interpolant.
     * @param samples Positions in the independent 2D world.
     * @param arc_length_parameters Strictly increasing native stations.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param closed True to require coincident active-domain endpoints.
     * @param tolerance Positive solve, seam, and definition tolerance.
     * @throws std::invalid_argument When samples, stations, or closure are invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    void adopt_to_points(
        const std::vector<point2<REAL>>& samples,
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

    /**
     * @brief Convert to the ordinary vector-backed 2D NURBS type.
     * @return Valid ordinary spline with identical geometry and metadata.
     * @throws std::invalid_argument When the current public definition is invalid.
     * @throws std::domain_error When an endpoint has near-zero homogeneous weight.
     * @throws std::bad_alloc When allocating vector storage fails.
     */
    [[nodiscard]] nurbs_spline2<REAL> to_nurbs_spline() const {
        return nurbs_spline2<REAL>(*this);
    }

private:
    struct layout_info {
        std::size_t control_count;
        std::size_t weight_offset;
        std::size_t knot_offset;
        std::size_t knot_count;
    };

    struct validated_state {
        layout_info layout;
        point2<REAL> start;
        point2<REAL> end;
    };

    [[nodiscard]] static std::valarray<REAL> standard_parameters(
        const point2<REAL>& start,
        const point2<REAL>& end,
        std::size_t control_point_count,
        std::size_t degree) {
        const std::size_t knot_count =
            nurbs_knot_count(degree, control_point_count);
        const std::size_t maximum = std::numeric_limits<std::size_t>::max();
        if (control_point_count >
            (maximum - knot_count) / std::size_t(3)) {
            throw std::overflow_error(
                "array-backed 2D NURBS parameter count overflows size_t");
        }
        std::vector<point2<REAL>> control_points(control_point_count);
        const REAL denominator =
            static_cast<REAL>(control_point_count - std::size_t(1));
        for (std::size_t index = 0; index < control_point_count; ++index) {
            const REAL fraction = static_cast<REAL>(index) / denominator;
            control_points[index] = lerp(start, end, fraction);
        }

        std::vector<REAL> knots(knot_count, REAL(0));
        const std::size_t span_count = control_point_count - degree;
        for (std::size_t span = 1; span < span_count; ++span) {
            knots[degree + span] =
                static_cast<REAL>(span) / static_cast<REAL>(span_count);
        }
        for (std::size_t index = control_point_count;
             index < knots.size(); ++index) {
            knots[index] = REAL(1);
        }
        return flatten_components(
            control_points,
            std::vector<REAL>(control_point_count, REAL(1)),
            knots,
            degree);
    }

    [[nodiscard]] static std::valarray<REAL> flatten_components(
        const std::vector<point2<REAL>>& control_points,
        const std::vector<REAL>& weights,
        const std::vector<REAL>& knots,
        std::size_t degree) {
        const std::size_t control_count = control_points.size();
        if (weights.size() != control_count) {
            throw std::invalid_argument(
                "array-backed 2D NURBS weights and control points must have equal size");
        }
        const std::size_t required_knot_count =
            nurbs_knot_count(degree, control_count);
        if (knots.size() != required_knot_count) {
            throw std::invalid_argument(
                "array-backed 2D NURBS knot count must equal control count + degree + 1");
        }
        const std::size_t maximum = std::numeric_limits<std::size_t>::max();
        if (control_count > (maximum - knots.size()) / std::size_t(3)) {
            throw std::overflow_error(
                "array-backed 2D NURBS parameter count overflows size_t");
        }
        const std::size_t parameter_count =
            std::size_t(3) * control_count + knots.size();
        std::valarray<REAL> result(REAL(0), parameter_count);
        for (std::size_t index = 0; index < control_count; ++index) {
            result[2 * index] = control_points[index].x;
            result[2 * index + 1] = control_points[index].y;
            result[2 * control_count + index] = weights[index];
        }
        const std::size_t knot_offset = std::size_t(3) * control_count;
        for (std::size_t index = 0; index < knots.size(); ++index) {
            result[knot_offset + index] = knots[index];
        }
        return result;
    }

    [[nodiscard]] layout_info checked_layout() const {
        if (degree_ == 0) {
            throw std::invalid_argument(
                "array-backed 2D NURBS degree must be positive");
        }
        if (degree_ == std::numeric_limits<std::size_t>::max() ||
            parameters.size() <= degree_) {
            throw std::invalid_argument(
                "array-backed 2D NURBS parameter array is too small for its degree");
        }
        const std::size_t non_degree_count =
            parameters.size() - degree_ - std::size_t(1);
        if (non_degree_count % std::size_t(4) != 0) {
            throw std::invalid_argument(
                "array-backed 2D NURBS parameter array has an invalid shape");
        }
        const std::size_t control_count =
            non_degree_count / std::size_t(4);
        if (control_count <= degree_) {
            throw std::invalid_argument(
                "array-backed 2D NURBS requires more control points than its degree");
        }
        const std::size_t weight_offset = std::size_t(2) * control_count;
        const std::size_t knot_offset = std::size_t(3) * control_count;
        const std::size_t knot_count = parameters.size() - knot_offset;
        return {control_count, weight_offset, knot_offset, knot_count};
    }

    [[nodiscard]] validated_state validated_definition() const {
        const layout_info layout = checked_layout();
        if (!(tolerance_ > REAL(0)) || !std::isfinite(tolerance_)) {
            throw std::invalid_argument(
                "array-backed 2D NURBS tolerance must be finite and positive");
        }
        for (std::size_t index = 0; index < layout.control_count; ++index) {
            const point2<REAL> point = control_point_unchecked(index);
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                throw std::invalid_argument(
                    "array-backed 2D NURBS control points must be finite");
            }
            const REAL current_weight =
                parameters[layout.weight_offset + index];
            if (!(current_weight > REAL(0)) ||
                !std::isfinite(current_weight)) {
                throw std::invalid_argument(
                    "array-backed 2D NURBS weights must be finite and positive");
            }
        }
        for (std::size_t index = 0; index < layout.knot_count; ++index) {
            const REAL current_knot = knot_unchecked(layout, index);
            if (!std::isfinite(current_knot)) {
                throw std::invalid_argument(
                    "array-backed 2D NURBS knots must be finite");
            }
            if (index > 0 &&
                current_knot < knot_unchecked(layout, index - 1)) {
                throw std::invalid_argument(
                    "array-backed 2D NURBS knots must be nondecreasing");
            }
        }
        const REAL lower = knot_unchecked(layout, degree_);
        const REAL upper = knot_unchecked(layout, layout.control_count);
        if (!(upper > lower)) {
            throw std::invalid_argument(
                "array-backed 2D NURBS active parameter domain must have positive length");
        }
        const point2<REAL> start = evaluate_validated(lower, layout);
        const point2<REAL> end = evaluate_validated(upper, layout);
        if (closed_ && distance(start, end) > closure_tolerance(layout)) {
            throw std::invalid_argument(
                "closed array-backed 2D NURBS endpoints must coincide");
        }
        return {layout, start, end};
    }

    [[nodiscard]] point2<REAL> control_point_unchecked(
        std::size_t index) const noexcept {
        return {parameters[2 * index], parameters[2 * index + 1]};
    }

    [[nodiscard]] REAL knot_unchecked(
        const layout_info& layout,
        std::size_t index) const noexcept {
        return parameters[layout.knot_offset + index];
    }

    [[nodiscard]] std::vector<point2<REAL>> control_points_for(
        const layout_info& layout) const {
        std::vector<point2<REAL>> result;
        result.reserve(layout.control_count);
        for (std::size_t index = 0; index < layout.control_count; ++index) {
            result.push_back(control_point_unchecked(index));
        }
        return result;
    }

    [[nodiscard]] std::vector<REAL> weights_for(
        const layout_info& layout) const {
        std::vector<REAL> result;
        result.reserve(layout.control_count);
        for (std::size_t index = 0; index < layout.control_count; ++index) {
            result.push_back(parameters[layout.weight_offset + index]);
        }
        return result;
    }

    [[nodiscard]] std::vector<REAL> knots_for(
        const layout_info& layout) const {
        std::vector<REAL> result;
        result.reserve(layout.knot_count);
        for (std::size_t index = 0; index < layout.knot_count; ++index) {
            result.push_back(knot_unchecked(layout, index));
        }
        return result;
    }

    [[nodiscard]] REAL closure_tolerance(
        const layout_info& layout) const noexcept {
        REAL scale = REAL(1);
        for (std::size_t index = 0; index < layout.control_count; ++index) {
            const point2<REAL> point = control_point_unchecked(index);
            scale = std::max({scale, std::abs(point.x), std::abs(point.y)});
        }
        return REAL(16) * tolerance_ * scale;
    }

    [[nodiscard]] REAL checked_parameter(
        REAL s,
        const layout_info& layout) const {
        if (!std::isfinite(s)) {
            throw std::out_of_range(
                "array-backed 2D NURBS parameter s must be finite");
        }
        const REAL lower = knot_unchecked(layout, degree_);
        const REAL upper = knot_unchecked(layout, layout.control_count);
        if (s < lower - tolerance_ || s > upper + tolerance_) {
            throw std::out_of_range(
                "array-backed 2D NURBS parameter s is outside the active knot domain");
        }
        return std::clamp(s, lower, upper);
    }

    [[nodiscard]] std::size_t find_span(
        REAL s,
        const layout_info& layout) const noexcept {
        const std::size_t n = layout.control_count - 1;
        if (s >= knot_unchecked(layout, n + 1)) {
            std::size_t span = n;
            while (span > degree_ &&
                   knot_unchecked(layout, span) ==
                       knot_unchecked(layout, span + 1)) {
                --span;
            }
            return span;
        }
        if (s <= knot_unchecked(layout, degree_)) {
            std::size_t span = degree_;
            while (span < n &&
                   knot_unchecked(layout, span) ==
                       knot_unchecked(layout, span + 1)) {
                ++span;
            }
            return span;
        }

        std::size_t lower = degree_;
        std::size_t upper = n + 1;
        std::size_t middle = (lower + upper) / 2;
        while (s < knot_unchecked(layout, middle) ||
               s >= knot_unchecked(layout, middle + 1)) {
            if (s < knot_unchecked(layout, middle)) {
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
        std::size_t order,
        const layout_info& layout) const {
        order = std::min(order, degree_);
        const std::size_t p = degree_;
        std::vector<std::vector<REAL>> ndu(
            p + 1, std::vector<REAL>(p + 1, REAL(0)));
        std::vector<REAL> left(p + 1, REAL(0));
        std::vector<REAL> right(p + 1, REAL(0));
        ndu[0][0] = REAL(1);

        for (std::size_t j = 1; j <= p; ++j) {
            left[j] = s - knot_unchecked(layout, span + 1 - j);
            right[j] = knot_unchecked(layout, span + j) - s;
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
                const std::ptrdiff_t rk =
                    static_cast<std::ptrdiff_t>(r) -
                    static_cast<std::ptrdiff_t>(k);
                const std::ptrdiff_t pk =
                    static_cast<std::ptrdiff_t>(p) -
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
                    derivative += work[destination][k] *
                        ndu[r][static_cast<std::size_t>(pk)];
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

    [[nodiscard]] std::array<vector2<REAL>, 4> rational_derivatives_at(
        REAL s,
        std::size_t requested_order,
        const layout_info& layout) const {
        requested_order = std::min<std::size_t>(requested_order, 3);
        const std::size_t span = find_span(s, layout);
        const std::size_t basis_order =
            std::min(requested_order, degree_);
        const auto basis_derivatives = basis_function_derivatives(
            span, s, basis_order, layout);

        std::array<vector2<REAL>, 4> numerator{};
        std::array<REAL, 4> weight_derivative{};
        for (std::size_t order = 0; order <= basis_order; ++order) {
            for (std::size_t local = 0; local <= degree_; ++local) {
                const std::size_t control_index = span - degree_ + local;
                const REAL current_weight =
                    parameters[layout.weight_offset + control_index];
                const REAL coefficient =
                    basis_derivatives[order][local] * current_weight;
                const point2<REAL> control =
                    control_point_unchecked(control_index);
                numerator[order] += coefficient *
                    vector2<REAL>{control.x, control.y};
                weight_derivative[order] += coefficient;
            }
        }
        if (std::abs(weight_derivative[0]) <= tolerance_) {
            throw std::domain_error(
                "array-backed 2D NURBS homogeneous weight is near zero");
        }

        std::array<vector2<REAL>, 4> result{};
        result[0] = numerator[0] / weight_derivative[0];
        for (std::size_t order = 1; order <= requested_order; ++order) {
            vector2<REAL> value = numerator[order];
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

    [[nodiscard]] point2<REAL> evaluate_validated(
        REAL s,
        const layout_info& layout) const {
        const vector2<REAL> value = rational_derivatives_at(s, 0, layout)[0];
        return {value.x, value.y};
    }

    std::size_t degree_;
    REAL tolerance_;
    bool closed_ = false;
};

template <std::floating_point REAL>
nurbs_spline2<REAL>::nurbs_spline2(
    const nurbs_arr_spline2<REAL>& spline)
    : nurbs_spline2(
          spline.get_control_points(),
          spline.get_weights(),
          spline.get_knots(),
          spline.degree(),
          spline.is_closed(),
          spline.tolerance()) {}

} // namespace nurbspath
