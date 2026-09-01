#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/nurbs_spline3.hpp"
#include "nurbspath/point3.hpp"
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
 * @brief Self-contained 3D NURBS curve backed by one public scalar valarray.
 *
 * The public `parameters` array stores point-major X/Y/Z control coordinates,
 * followed by one weight per control point and then the complete knot vector.
 * For `C` control points and degree `p`, its size is `5*C + p + 1`, with
 * offsets `3*i+c` for control coordinate `c`, `3*C+i` for weight `i`, and
 * `4*C+j` for knot `j`. Degree, closure, and tolerance remain separately
 * typed metadata and are not scalar definition parameters.
 *
 * Direct replacement or mutation of `parameters` is deliberately supported
 * and may temporarily make the object invalid. Every geometric operation
 * validates the current array before indexing or evaluation. The class keeps
 * no endpoint cache, so direct parameter changes are immediately reflected by
 * `get_start()`, `get_end()`, and evaluation. References into `parameters`
 * are invalidated when the array is replaced.
 *
 * This is a distinct spline implementation. It neither inherits from nor
 * delegates evaluation to `nurbs_spline3`; conversion constructs a separate
 * validated ordinary spline only when explicitly requested.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class nurbs_arr_spline3 {
public:
    /** @brief Scalar type stored by this spline. */
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
     * @brief Flattened control-coordinate, weight, and knot storage.
     *
     * The array can be copied, moved, replaced, and edited directly. Such an
     * edit is not validated until `validate()` or a geometric operation is
     * called.
     */
    std::valarray<REAL> parameters;

    /**
     * @brief Construct a standard open 3D spline with explicit tolerance.
     *
     * Control points are distributed linearly from `start` through `end`,
     * every weight is one, and standard open-clamped knots use the native
     * domain `[0, distance(start, end)]`.
     *
     * @param start First control point and active-domain endpoint.
     * @param end Final control point and active-domain endpoint.
     * @param control_point_count Number of linearly distributed controls;
     * must exceed `degree`.
     * @param degree Positive polynomial degree.
     * @param tolerance Positive definition and parameter-boundary tolerance.
     * @throws std::invalid_argument When the endpoints, counts, degree,
     * distance, generated definition, or tolerance are invalid.
     * @throws std::overflow_error When the required knot or parameter count is
     * not representable by `std::size_t`.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    nurbs_arr_spline3(
        const point3<REAL>& start,
        const point3<REAL>& end,
        std::size_t control_point_count,
        std::size_t degree,
        REAL tolerance)
        : nurbs_arr_spline3(
              start,
              end,
              control_point_count,
              degree,
              false,
              tolerance) {}

    /**
     * @brief Construct a standard open or closed spline between two 3D points.
     *
     * Control points are distributed linearly from `start` through `end`
     * with `lerp`, every weight is one, and standard open-clamped knots use
     * the native domain `[0, distance(start, end)]`. A closed definition
     * must also satisfy the ordinary endpoint-coincidence contract.
     *
     * @tparam CLOSED Exact Boolean closure-flag type; defaults to `bool`.
     * @param start First control point and active-domain endpoint.
     * @param end Final control point and active-domain endpoint.
     * @param control_point_count Number of linearly distributed controls;
     * must exceed `degree`.
     * @param degree Positive polynomial degree.
     * @param closed True when the active-domain endpoints must coincide;
     * defaults to false.
     * @param tolerance Positive definition and parameter-boundary tolerance;
     * defaults to `64 * std::numeric_limits<REAL>::epsilon()`.
     * @throws std::invalid_argument When the endpoints, counts, degree,
     * distance, generated definition, closure, or tolerance are invalid.
     * @throws std::overflow_error When the required knot or parameter count is
     * not representable by `std::size_t`.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    template <std::same_as<bool> CLOSED = bool>
    nurbs_arr_spline3(
        const point3<REAL>& start,
        const point3<REAL>& end,
        std::size_t control_point_count,
        std::size_t degree,
        CLOSED closed = false,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : nurbs_arr_spline3(
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
     * @brief Construct an open array spline from flattened scalar parameters.
     * @param parameter_values Point coordinates, weights, and knots in the
     * documented flattened order.
     * @param degree Positive polynomial degree below the derived control-point
     * count.
     * @param tolerance Positive definition and parameter-boundary tolerance.
     * @throws std::invalid_argument When the array shape, degree, values, or
     * tolerance are invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    nurbs_arr_spline3(
        std::valarray<REAL> parameter_values,
        std::size_t degree,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : nurbs_arr_spline3(
              std::move(parameter_values), degree, false, tolerance) {}

    /**
     * @brief Construct an open or closed array spline from flattened scalars.
     * @tparam CLOSED Exact Boolean closure-flag type.
     * @param parameter_values Point coordinates, weights, and knots in the
     * documented flattened order.
     * @param degree Positive polynomial degree below the derived control-point
     * count.
     * @param closed True when the two active-domain endpoints must coincide.
     * @param tolerance Positive validation and parameter-boundary tolerance.
     * @throws std::invalid_argument When the definition or closed seam is
     * invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    template <std::same_as<bool> CLOSED>
    nurbs_arr_spline3(
        std::valarray<REAL> parameter_values,
        std::size_t degree,
        CLOSED closed,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : parameters(std::move(parameter_values)),
          degree_(degree),
          tolerance_(tolerance),
          closed_(closed) {
        validate();
    }

    /**
     * @brief Construct an open array spline from ordinary vector collections.
     * @param control_points Finite world-space control points.
     * @param weights Finite positive weight for every control point.
     * @param knots Complete finite nondecreasing knot vector.
     * @param degree Positive polynomial degree below the control-point count.
     * @param tolerance Positive definition and parameter-boundary tolerance.
     * @throws std::invalid_argument When counts, degree, values, or tolerance
     * are invalid.
     * @throws std::overflow_error When the required knot or parameter count is
     * not representable by `std::size_t`.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    nurbs_arr_spline3(
        std::vector<point3<REAL>> control_points,
        std::vector<REAL> weights,
        std::vector<REAL> knots,
        std::size_t degree,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : nurbs_arr_spline3(
              std::move(control_points),
              std::move(weights),
              std::move(knots),
              degree,
              false,
              tolerance) {}

    /**
     * @brief Construct an open or closed array spline from vector collections.
     * @tparam CLOSED Exact Boolean closure-flag type.
     * @param control_points Finite world-space control points.
     * @param weights Finite positive weight for every control point.
     * @param knots Complete finite nondecreasing knot vector.
     * @param degree Positive polynomial degree below the control-point count.
     * @param closed True when the two active-domain endpoints must coincide.
     * @param tolerance Positive validation and parameter-boundary tolerance.
     * @throws std::invalid_argument When the definition or closed seam is
     * invalid.
     * @throws std::overflow_error When the required knot or parameter count is
     * not representable by `std::size_t`.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    template <std::same_as<bool> CLOSED>
    nurbs_arr_spline3(
        std::vector<point3<REAL>> control_points,
        std::vector<REAL> weights,
        std::vector<REAL> knots,
        std::size_t degree,
        CLOSED closed,
        REAL tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon())
        : nurbs_arr_spline3(
              pack_parameters(control_points, weights, knots, degree),
              degree,
              closed,
              tolerance) {}

    /**
     * @brief Copy an ordinary 3D NURBS into independent array storage.
     * @param spline Valid ordinary spline whose complete state is copied.
     * @throws std::overflow_error When the flattened parameter count is not
     * representable by `std::size_t`.
     * @throws std::bad_alloc When allocating the new array fails.
     */
    explicit nurbs_arr_spline3(const nurbs_spline3<REAL>& spline)
        : nurbs_arr_spline3(
              spline.get_control_points(),
              spline.get_weights(),
              spline.get_knots(),
              spline.degree(),
              spline.is_closed(),
              spline.tolerance()) {}

    /**
     * @brief Get the current flattened scalar count.
     * @return Number of values in `parameters`, including an invalid draft.
     */
    [[nodiscard]] std::size_t number_of_parameters() const noexcept {
        return parameters.size();
    }

    /**
     * @brief Derive the current control-point count from shape and degree.
     * @return Number of control points and weights.
     * @throws std::invalid_argument When the current array has no valid 3D
     * flattened shape for the stored degree.
     */
    [[nodiscard]] std::size_t control_point_count() const {
        return checked_layout().control_count;
    }

    /**
     * @brief Derive the current weight count.
     * @return Number of rational weights, equal to `control_point_count()`.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] std::size_t weight_count() const {
        return checked_layout().control_count;
    }

    /**
     * @brief Derive the current knot count.
     * @return Complete knot-vector length.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] std::size_t knot_count() const {
        return checked_layout().knot_count;
    }

    /**
     * @brief Get the flattened control-coordinate index range.
     *
     * Each X, Y, and Z coordinate occupies one index, so the returned
     * half-open range contains three indices per control point. Its values
     * can be used directly to access `parameters`.
     *
     * @return Lazy ascending range of valid control-coordinate scalar indices.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] parameter_index_range get_control_point_count_range() const {
        const layout current = checked_layout();
        return std::views::iota(std::size_t(0), current.weight_offset);
    }

    /**
     * @brief Get the flattened weight index range.
     *
     * The returned values are indices into `parameters`, not weight-relative
     * indices.
     *
     * @return Lazy ascending range of valid flattened weight indices.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] parameter_index_range get_weight_count_range() const {
        const layout current = checked_layout();
        return std::views::iota(current.weight_offset, current.knot_offset);
    }

    /**
     * @brief Get the flattened knot index range.
     *
     * The returned values are indices into `parameters`, not knot-relative
     * indices.
     *
     * @return Lazy ascending range of valid flattened knot indices.
     * @throws std::invalid_argument When the current array shape is invalid.
     */
    [[nodiscard]] parameter_index_range get_knot_count_range() const {
        const layout current = checked_layout();
        return std::views::iota(current.knot_offset, parameters.size());
    }

    /** @brief Get the polynomial degree. @return Immutable spline degree. */
    [[nodiscard]] std::size_t degree() const noexcept { return degree_; }

    /** @brief Get the definition tolerance. @return Positive tolerance. */
    [[nodiscard]] REAL tolerance() const noexcept { return tolerance_; }

    /** @brief Report closure state. @return True when closure is required. */
    [[nodiscard]] bool is_closed() const noexcept { return closed_; }

    /**
     * @brief Copy all control points out of scalar storage.
     * @return Point vector in control-point order.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::bad_alloc When allocating the vector fails.
     */
    [[nodiscard]] std::vector<point3<REAL>> get_control_points() const {
        const layout current = checked_layout();
        std::vector<point3<REAL>> result;
        result.reserve(current.control_count);
        for (std::size_t index = 0; index < current.control_count; ++index) {
            result.push_back(control_point_unchecked(index));
        }
        return result;
    }

    /**
     * @brief Copy all rational weights out of scalar storage.
     * @return Weight vector in control-point order.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::bad_alloc When allocating the vector fails.
     */
    [[nodiscard]] std::vector<REAL> get_weights() const {
        const layout current = checked_layout();
        std::vector<REAL> result;
        result.reserve(current.control_count);
        for (std::size_t index = 0; index < current.control_count; ++index) {
            result.push_back(parameters[current.weight_offset + index]);
        }
        return result;
    }

    /**
     * @brief Copy the complete knot vector out of scalar storage.
     * @return Knot vector in stored order.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::bad_alloc When allocating the vector fails.
     */
    [[nodiscard]] std::vector<REAL> get_knots() const {
        const layout current = checked_layout();
        std::vector<REAL> result;
        result.reserve(current.knot_count);
        for (std::size_t index = 0; index < current.knot_count; ++index) {
            result.push_back(parameters[current.knot_offset + index]);
        }
        return result;
    }

    /**
     * @brief Get one flattened scalar parameter.
     * @param index Zero-based scalar index.
     * @return Stored parameter value.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::out_of_range When `index` is outside `parameters`.
     */
    [[nodiscard]] REAL get_parameter(std::size_t index) const {
        static_cast<void>(checked_layout());
        if (index >= parameters.size()) {
            throw std::out_of_range(
                "3D array NURBS parameter index is out of range");
        }
        return parameters[index];
    }

    /**
     * @brief Get the descriptive name of one flattened scalar.
     * @param index Zero-based scalar index.
     * @return `P[i].x`, `P[i].y`, `P[i].z`, `W[i]`, or `K[i]`.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::out_of_range When `index` is outside `parameters`.
     * @throws std::bad_alloc When allocating the returned string fails.
     */
    [[nodiscard]] std::string parameter_name(std::size_t index) const {
        const layout current = checked_layout();
        if (index < current.weight_offset) {
            constexpr char coordinates[] = {'x', 'y', 'z'};
            return "P[" + std::to_string(index / 3) + "]." +
                   coordinates[index % 3];
        }
        index -= current.weight_offset;
        if (index < current.control_count) {
            return "W[" + std::to_string(index) + "]";
        }
        index -= current.control_count;
        if (index < current.knot_count) {
            return "K[" + std::to_string(index) + "]";
        }
        throw std::out_of_range(
            "3D array NURBS parameter index is out of range");
    }

    /**
     * @brief Atomically validate and replace one scalar parameter.
     * @param index Zero-based scalar index.
     * @param value Candidate scalar value.
     * @return True when the candidate is valid and committed; false for an
     * invalid shape, out-of-range index, or rejected candidate definition.
     * @throws std::bad_alloc When allocating a detached candidate fails.
     */
    [[nodiscard]] bool set_parameter(std::size_t index, REAL value) {
        try {
            static_cast<void>(checked_layout());
        } catch (const std::invalid_argument&) {
            return false;
        }
        if (index >= parameters.size()) {
            return false;
        }

        std::valarray<REAL> candidate = parameters;
        candidate[index] = value;
        try {
            set_parameters(std::move(candidate));
        } catch (const std::invalid_argument&) {
            return false;
        } catch (const std::domain_error&) {
            return false;
        }
        return true;
    }

    /**
     * @brief Materialize one control point from three stored scalars.
     * @param index Zero-based control-point index.
     * @return Point value copied from X/Y/Z storage.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::out_of_range When `index` is outside the control points.
     */
    [[nodiscard]] point3<REAL> get_control_point(std::size_t index) const {
        const layout current = checked_layout();
        if (index >= current.control_count) {
            throw std::out_of_range(
                "3D array NURBS control-point index is out of range");
        }
        return control_point_unchecked(index);
    }

    /**
     * @brief Get one rational weight.
     * @param index Zero-based weight index.
     * @return Stored weight value.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::out_of_range When `index` is outside the weights.
     */
    [[nodiscard]] REAL get_weight(std::size_t index) const {
        const layout current = checked_layout();
        if (index >= current.control_count) {
            throw std::out_of_range(
                "3D array NURBS weight index is out of range");
        }
        return parameters[current.weight_offset + index];
    }

    /**
     * @brief Get one knot.
     * @param index Zero-based knot index.
     * @return Stored knot value.
     * @throws std::invalid_argument When the current array shape is invalid.
     * @throws std::out_of_range When `index` is outside the knot vector.
     */
    [[nodiscard]] REAL get_knot(std::size_t index) const {
        const layout current = checked_layout();
        if (index >= current.knot_count) {
            throw std::out_of_range(
                "3D array NURBS knot index is out of range");
        }
        return parameters[current.knot_offset + index];
    }

    /**
     * @brief Validate the current public array and all spline metadata.
     * @throws std::invalid_argument When shape, degree, scalar values, active
     * domain, tolerance, or a required closed seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void validate() const {
        static_cast<void>(validated_layout());
    }

    /**
     * @brief Atomically replace the complete flattened parameter array.
     * @param parameter_values Candidate point-coordinate, weight, and knot
     * values in flattened order.
     * @throws std::invalid_argument When the candidate definition or seam is
     * invalid for the current degree, closure state, or tolerance.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_parameters(std::valarray<REAL> parameter_values) {
        nurbs_arr_spline3 replacement(
            std::move(parameter_values), degree_, closed_, tolerance_);
        *this = std::move(replacement);
    }

    /**
     * @brief Atomically copy and replace the complete flattened parameter array.
     *
     * Exactly `number_of_parameters()` consecutive values are copied before
     * validation, so `parameter_values` may point to the first element of this
     * object's current `parameters` storage. This overload cannot verify the
     * source buffer length; passing a non-null pointer to fewer readable values
     * has undefined behavior.
     *
     * @param parameter_values Pointer to at least `number_of_parameters()`
     * point-coordinate, weight, and knot values in flattened order.
     * @throws std::invalid_argument When `parameter_values` is null or the
     * copied candidate definition is invalid for the current metadata.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     * @throws std::bad_alloc When allocating the detached candidate fails.
     */
    void set_parameters(const REAL* parameter_values) {
        if (parameter_values == nullptr) {
            throw std::invalid_argument(
                "3D array NURBS parameter pointer must not be null");
        }

        // Detach before validation and assignment because parameter_values
        // may alias the current public valarray storage.
        std::valarray<REAL> candidate(
            parameter_values, parameters.size());
        set_parameters(std::move(candidate));
    }

    /**
     * @brief Atomically replace one control point.
     * @param index Zero-based control-point index.
     * @param point Candidate finite world-space point.
     * @throws std::invalid_argument When the resulting definition or seam is
     * invalid.
     * @throws std::out_of_range When `index` is outside the control points.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_control_point(std::size_t index, const point3<REAL>& point) {
        const layout current = checked_layout();
        if (index >= current.control_count) {
            throw std::out_of_range(
                "3D array NURBS control-point index is out of range");
        }
        std::valarray<REAL> candidate = parameters;
        const std::size_t offset = index * std::size_t(3);
        candidate[offset] = point.x;
        candidate[offset + 1] = point.y;
        candidate[offset + 2] = point.z;
        set_parameters(std::move(candidate));
    }

    /**
     * @brief Atomically replace one rational weight.
     * @param index Zero-based weight index.
     * @param value Candidate finite positive weight.
     * @throws std::invalid_argument When the resulting definition is invalid.
     * @throws std::out_of_range When `index` is outside the weights.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_weight(std::size_t index, REAL value) {
        const layout current = checked_layout();
        if (index >= current.control_count) {
            throw std::out_of_range(
                "3D array NURBS weight index is out of range");
        }
        std::valarray<REAL> candidate = parameters;
        candidate[current.weight_offset + index] = value;
        set_parameters(std::move(candidate));
    }

    /**
     * @brief Atomically replace one knot.
     * @param index Zero-based knot index.
     * @param value Candidate finite knot value.
     * @throws std::invalid_argument When the resulting knot vector, domain, or
     * seam is invalid.
     * @throws std::out_of_range When `index` is outside the knots.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_knot(std::size_t index, REAL value) {
        const layout current = checked_layout();
        if (index >= current.knot_count) {
            throw std::out_of_range(
                "3D array NURBS knot index is out of range");
        }
        std::valarray<REAL> candidate = parameters;
        candidate[current.knot_offset + index] = value;
        set_parameters(std::move(candidate));
    }

    /**
     * @brief Atomically replace all control points while retaining other data.
     * @param control_points Candidate finite world-space control points.
     * @throws std::invalid_argument When counts or the resulting definition are
     * invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_control_points(std::vector<point3<REAL>> control_points) {
        set_definition(
            std::move(control_points),
            get_weights(),
            get_knots(),
            closed_,
            tolerance_);
    }

    /**
     * @brief Atomically replace all weights while retaining other data.
     * @param weight_values Candidate finite positive weights.
     * @throws std::invalid_argument When counts or the resulting definition are
     * invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_weights(std::vector<REAL> weight_values) {
        set_definition(
            get_control_points(), std::move(weight_values), get_knots(),
            closed_, tolerance_);
    }

    /**
     * @brief Atomically replace the complete knot vector.
     * @param knot_values Candidate finite nondecreasing knot vector.
     * @throws std::invalid_argument When counts, order, domain, or the closed
     * seam are invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_knots(std::vector<REAL> knot_values) {
        set_definition(
            get_control_points(), get_weights(), std::move(knot_values),
            closed_, tolerance_);
    }

    /**
     * @brief Atomically replace all vector definition fields and metadata.
     *
     * The polynomial degree is retained. Unlike direct public array editing,
     * this operation validates a complete detached candidate before commit.
     *
     * @param control_points Candidate world-space control points.
     * @param weight_values Candidate positive rational weights.
     * @param knot_values Candidate nondecreasing knot vector.
     * @param closed True to require coincident active-domain endpoints.
     * @param tolerance Candidate finite positive tolerance.
     * @throws std::invalid_argument When the candidate definition or seam is
     * invalid.
     * @throws std::overflow_error When the flattened parameter count is not
     * representable by `std::size_t`.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_definition(
        std::vector<point3<REAL>> control_points,
        std::vector<REAL> weight_values,
        std::vector<REAL> knot_values,
        bool closed,
        REAL tolerance) {
        nurbs_arr_spline3 replacement(
            std::move(control_points),
            std::move(weight_values),
            std::move(knot_values),
            degree_,
            closed,
            tolerance);
        *this = std::move(replacement);
    }

    /**
     * @brief Atomically change the validation tolerance.
     * @param tolerance Candidate finite positive tolerance.
     * @throws std::invalid_argument When the tolerance or resulting seam is
     * invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_tolerance(REAL tolerance) {
        nurbs_arr_spline3 replacement(
            parameters, degree_, closed_, tolerance);
        *this = std::move(replacement);
    }

    /**
     * @brief Atomically change the closed-seam requirement.
     * @param closed True to require coincident active-domain endpoints.
     * @throws std::invalid_argument When enabling closure on an open seam or
     * when the current public array is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_closed(bool closed) {
        nurbs_arr_spline3 replacement(
            parameters, degree_, closed, tolerance_);
        *this = std::move(replacement);
    }

    /**
     * @brief Set standard open-clamped knots on `[0, s_end]`.
     * @param s_end Finite positive upper active-domain bound.
     * @throws std::invalid_argument When the bound, generated spans,
     * definition, or closed seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_standard_knots(REAL s_end) {
        set_standard_knots(REAL(0), s_end);
    }

    /**
     * @brief Set standard open-clamped knots on `[s_start, s_end]`.
     * @param s_start Finite lower active-domain bound.
     * @param s_end Finite upper active-domain bound strictly greater than
     * `s_start`.
     * @throws std::invalid_argument When the bounds, generated spans,
     * definition, or closed seam is invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     */
    void set_standard_knots(REAL s_start, REAL s_end) {
        const layout current = validated_layout();
        if (!std::isfinite(s_start) || !std::isfinite(s_end) ||
            !(s_end > s_start) || !std::isfinite(s_end - s_start)) {
            throw std::invalid_argument(
                "standard NURBS knot domain must have finite positive length");
        }

        const std::size_t span_count = current.control_count - degree_;
        std::vector<REAL> standard_knots(current.knot_count, s_start);
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
                    "standard NURBS knot spans must be numerically usable");
            }
            standard_knots[degree_ + span] = interior_knot;
            previous_knot = interior_knot;
        }
        if (!has_usable_span(previous_knot, s_end)) {
            throw std::invalid_argument(
                "standard NURBS knot spans must be numerically usable");
        }
        for (std::size_t index = current.control_count;
             index < standard_knots.size(); ++index) {
            standard_knots[index] = s_end;
        }
        set_knots(std::move(standard_knots));
    }

    /**
     * @brief Compute the current point at the start of the active domain.
     * @return Evaluated start point by value; no endpoint is cached.
     * @throws std::invalid_argument When the current public array is invalid.
     * @throws std::domain_error When homogeneous weight is near zero.
     */
    [[nodiscard]] point3<REAL> get_start() const {
        const layout current = validated_layout();
        return point_from_vector(rational_derivatives_at_unchecked(
            s_min_unchecked(current), 0, current)[0]);
    }

    /**
     * @brief Compute the current point at the end of the active domain.
     * @return Evaluated end point by value; no endpoint is cached.
     * @throws std::invalid_argument When the current public array is invalid.
     * @throws std::domain_error When homogeneous weight is near zero.
     */
    [[nodiscard]] point3<REAL> get_end() const {
        const layout current = validated_layout();
        return point_from_vector(rational_derivatives_at_unchecked(
            s_max_unchecked(current), 0, current)[0]);
    }

    /**
     * @brief Get the current lower active parameter bound.
     * @return Knot at index `degree()`.
     * @throws std::invalid_argument When the current public array is invalid.
     * @throws std::domain_error When endpoint validation has near-zero
     * homogeneous weight.
     */
    [[nodiscard]] REAL s_min() const {
        const layout current = validated_layout();
        return s_min_unchecked(current);
    }

    /**
     * @brief Get the current upper active parameter bound.
     * @return Knot at index `control_point_count()`.
     * @throws std::invalid_argument When the current public array is invalid.
     * @throws std::domain_error When endpoint validation has near-zero
     * homogeneous weight.
     */
    [[nodiscard]] REAL s_max() const {
        const layout current = validated_layout();
        return s_max_unchecked(current);
    }

    /**
     * @brief Evaluate position and the first two analytic derivatives.
     * @param s Finite parameter in the tolerated active knot domain.
     * @return Position and derivatives with respect to native `s`.
     * @throws std::invalid_argument When the current public definition is
     * invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When homogeneous weight is near zero.
     */
    [[nodiscard]] spline_derivatives3<REAL> derivatives_at(REAL s) const {
        const layout current = validated_layout();
        const auto derivatives =
            rational_derivatives_at_unchecked(s, 2, current);
        return {
            point_from_vector(derivatives[0]),
            derivatives[1],
            derivatives[2]};
    }

    /**
     * @brief Evaluate a world-space curve point.
     * @param s Parameter in the active knot domain.
     * @return Evaluated point.
     * @throws std::invalid_argument When the current public definition is
     * invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When homogeneous weight is near zero.
     */
    [[nodiscard]] point3<REAL> evaluate(REAL s) const {
        return derivatives_at(s).point;
    }

    /**
     * @brief Evaluate a world-space curve point.
     * @param s Parameter in the active knot domain.
     * @return Evaluated point.
     * @throws std::invalid_argument When the current public definition is
     * invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When homogeneous weight is near zero.
     */
    [[nodiscard]] point3<REAL> point_at(REAL s) const { return evaluate(s); }

    /**
     * @brief Evaluate the first analytic derivative.
     * @param s Parameter in the active knot domain.
     * @return First derivative with respect to native `s`.
     * @throws std::invalid_argument When the current public definition is
     * invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When homogeneous weight is near zero.
     */
    [[nodiscard]] vector3<REAL> first_derivative(REAL s) const {
        return derivatives_at(s).first;
    }

    /**
     * @brief Evaluate the second analytic derivative.
     * @param s Parameter in the active knot domain.
     * @return Second derivative with respect to native `s`.
     * @throws std::invalid_argument When the current public definition is
     * invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When homogeneous weight is near zero.
     */
    [[nodiscard]] vector3<REAL> second_derivative(REAL s) const {
        return derivatives_at(s).second;
    }

    /**
     * @brief Evaluate the third analytic rational derivative.
     * @param s Parameter in the active knot domain.
     * @return Third derivative with respect to native `s`.
     * @throws std::invalid_argument When the current public definition is
     * invalid.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When homogeneous weight is near zero.
     */
    [[nodiscard]] vector3<REAL> third_derivative(REAL s) const {
        const layout current = validated_layout();
        return rational_derivatives_at_unchecked(s, 3, current)[3];
    }

    /**
     * @brief Evaluate a unit tangent.
     * @param s Parameter in the active knot domain.
     * @param tangent_tolerance Minimum accepted first-derivative length.
     * @return Normalized first derivative.
     * @throws std::invalid_argument When the current public definition is
     * invalid.
     * @throws std::out_of_range When `s` is outside the active domain.
     * @throws std::domain_error When a homogeneous weight or tangent length is
     * near zero.
     */
    [[nodiscard]] vector3<REAL> tangent(
        REAL s,
        REAL tangent_tolerance = vector3<REAL>::default_tolerance()) const {
        return first_derivative(s).normalized(tangent_tolerance);
    }

    /**
     * @brief Evaluate unsigned geometric curvature in world space.
     *
     * Curvature is calculated analytically as
     * `length(C'(s) cross C''(s)) / length(C'(s))^3` and is independent of
     * the scale of the native parameter while the analytic derivatives remain
     * representable in `REAL`. At an internal knot without second-derivative
     * continuity, the result uses the right-hand nonempty span; `s_max()` uses
     * the left-hand nonempty span. This one-sided value does not represent
     * curvature at a corner.
     *
     * @param s Finite parameter in the active knot domain.
     * @param tangent_tolerance Nonnegative finite minimum accepted
     * first-derivative length.
     * @return Nonnegative curvature magnitude in inverse world units.
     * @throws std::invalid_argument When the current definition is invalid or
     * `tangent_tolerance` is negative or non-finite.
     * @throws std::out_of_range When `s` is non-finite or outside the domain.
     * @throws std::domain_error When a homogeneous weight is near zero or the
     * first-derivative length is non-finite or not longer than
     * `tangent_tolerance`, or finite curvature cannot be represented.
     */
    [[nodiscard]] REAL curvature(
        REAL s,
        REAL tangent_tolerance = vector3<REAL>::default_tolerance()) const {
        if (!(tangent_tolerance >= REAL(0)) ||
            !std::isfinite(tangent_tolerance)) {
            throw std::invalid_argument(
                "curvature tangent tolerance must be finite and nonnegative");
        }

        const spline_derivatives3<REAL> derivatives = derivatives_at(s);
        const REAL speed = derivatives.first.length();
        if (!std::isfinite(speed) || !(speed > tangent_tolerance)) {
            throw std::domain_error(
                "curvature requires a finite nonzero 3D spline derivative");
        }
        const vector3<REAL> unit = derivatives.first / speed;
        const vector3<REAL> scaled_second = derivatives.second / speed;
        const REAL result = unit.cross(scaled_second).length() / speed;
        if (!std::isfinite(result)) {
            throw std::domain_error("cannot represent finite 3D spline curvature");
        }
        return result;
    }

    /**
     * @brief Estimate arc length using a uniform-native-parameter polyline.
     * @param segment_count Positive number of approximation segments.
     * @return Approximate world-space arc length.
     * @throws std::invalid_argument When the definition is invalid or
     * `segment_count` is zero.
     * @throws std::domain_error When homogeneous weight is near zero.
     */
    [[nodiscard]] REAL approximate_arc_length(
        std::size_t segment_count = 512) const {
        if (segment_count == 0) {
            throw std::invalid_argument("segment_count must be positive");
        }
        const layout current = validated_layout();
        const REAL lower = s_min_unchecked(current);
        const REAL upper = s_max_unchecked(current);
        point3<REAL> previous = point_from_vector(
            rational_derivatives_at_unchecked(lower, 0, current)[0]);
        REAL length = REAL(0);
        for (std::size_t index = 0; index < segment_count; ++index) {
            const REAL fraction = static_cast<REAL>(index + 1) /
                                  static_cast<REAL>(segment_count);
            const REAL s = lower + fraction * (upper - lower);
            const point3<REAL> current_point = point_from_vector(
                rational_derivatives_at_unchecked(s, 0, current)[0]);
            length += distance(previous, current_point);
            previous = current_point;
        }
        return length;
    }

    /**
     * @brief Build an open unit-weight spline interpolating measured samples.
     * @param samples World-space positions to interpolate exactly.
     * @param arc_length_parameters Strictly increasing native stations.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param tolerance Positive solve and definition tolerance.
     * @return Self-contained interpolating array spline.
     * @throws std::invalid_argument When sample data or parameters are invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    [[nodiscard]] static nurbs_arr_spline3 interpolate(
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
     * @brief Build an open or closed unit-weight interpolating array spline.
     * @param samples World-space positions to interpolate exactly.
     * @param arc_length_parameters Strictly increasing native stations.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param closed True to require coincident active-domain endpoints.
     * @param tolerance Positive solve, seam, and definition tolerance.
     * @return Self-contained interpolating array spline.
     * @throws std::invalid_argument When sample data or closure is invalid.
     * @throws std::domain_error When the interpolation system is singular.
     */
    [[nodiscard]] static nurbs_arr_spline3 interpolate(
        const std::vector<point3<REAL>>& samples,
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
            throw std::invalid_argument(
                "interpolation degree must be positive");
        }
        if (!(tolerance > REAL(0)) || !std::isfinite(tolerance)) {
            throw std::invalid_argument(
                "interpolation tolerance must be finite and positive");
        }
        for (const point3<REAL>& sample : samples) {
            if (!std::isfinite(sample.x) || !std::isfinite(sample.y) ||
                !std::isfinite(sample.z)) {
                throw std::invalid_argument(
                    "interpolation samples must be finite");
            }
        }
        for (const REAL station : arc_length_parameters) {
            if (!std::isfinite(station)) {
                throw std::invalid_argument(
                    "arc-length parameters must be finite");
            }
        }
        for (std::size_t index = 1;
             index < arc_length_parameters.size(); ++index) {
            if (!(arc_length_parameters[index] >
                  arc_length_parameters[index - 1])) {
                throw std::invalid_argument(
                    "arc-length parameters must be strictly increasing");
            }
        }
        if (closed &&
            distance(samples.front(), samples.back()) > tolerance) {
            throw std::invalid_argument(
                "closed interpolation requires coincident first and final samples");
        }

        const std::size_t point_count = samples.size();
        const std::size_t degree =
            std::min(requested_degree, point_count - 1);
        const std::size_t n = point_count - 1;
        std::vector<REAL> knot_values(
            nurbs_knot_count(degree, point_count), REAL(0));

        std::fill_n(
            knot_values.begin(), degree + 1,
            arc_length_parameters.front());
        std::fill_n(
            knot_values.end() - static_cast<std::ptrdiff_t>(degree + 1),
            degree + 1,
            arc_length_parameters.back());
        for (std::size_t j = 1; j <= n - degree; ++j) {
            REAL sum = REAL(0);
            for (std::size_t index = j; index < j + degree; ++index) {
                sum += arc_length_parameters[index];
            }
            knot_values[j + degree] = sum / static_cast<REAL>(degree);
        }

        std::vector<REAL> unit_weights(point_count, REAL(1));
        const nurbs_arr_spline3 seed(
            samples, unit_weights, knot_values, degree, tolerance);
        const layout seed_layout = seed.checked_layout();
        std::vector<std::vector<REAL>> matrix(
            point_count, std::vector<REAL>(point_count, REAL(0)));
        for (std::size_t row = 0; row < point_count; ++row) {
            const REAL s = arc_length_parameters[row];
            const std::size_t span = seed.find_span_unchecked(
                s, seed_layout);
            const auto values = seed.basis_function_derivatives_unchecked(
                span, s, 0, seed_layout).front();
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
        const auto x_controls =
            solve_linear_system(matrix, x_values, tolerance);
        const auto y_controls =
            solve_linear_system(matrix, y_values, tolerance);
        const auto z_controls =
            solve_linear_system(matrix, z_values, tolerance);

        std::vector<point3<REAL>> control_points(point_count);
        for (std::size_t index = 0; index < point_count; ++index) {
            control_points[index] = {
                x_controls[index], y_controls[index], z_controls[index]};
        }
        return nurbs_arr_spline3(
            std::move(control_points),
            std::move(unit_weights),
            std::move(knot_values),
            degree,
            closed,
            tolerance);
    }

    /**
     * @brief Replace this spline with an open interpolant through samples.
     * @param samples World-space positions to interpolate exactly.
     * @param arc_length_parameters Strictly increasing native stations.
     * @param requested_degree Desired positive degree, reduced when necessary.
     * @param tolerance Positive solve and definition tolerance.
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
     * @brief Replace this spline with an open or closed interpolant.
     * @param samples World-space positions to interpolate exactly.
     * @param arc_length_parameters Strictly increasing native stations.
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

    /**
     * @brief Create an independent ordinary 3D NURBS spline.
     * @return Validated ordinary spline with identical geometry and metadata.
     * @throws std::invalid_argument When the current public definition is
     * invalid.
     * @throws std::domain_error When endpoint evaluation has near-zero
     * homogeneous weight.
     * @throws std::bad_alloc When allocating ordinary vector storage fails.
     */
    [[nodiscard]] nurbs_spline3<REAL> to_nurbs_spline() const {
        return nurbs_spline3<REAL>(*this);
    }

private:
    struct layout {
        std::size_t control_count;
        std::size_t weight_offset;
        std::size_t knot_offset;
        std::size_t knot_count;
    };

    [[nodiscard]] static std::valarray<REAL> standard_parameters(
        const point3<REAL>& start,
        const point3<REAL>& end,
        std::size_t control_point_count,
        std::size_t degree) {
        const std::size_t knot_count =
            nurbs_knot_count(degree, control_point_count);
        const std::size_t maximum = std::numeric_limits<std::size_t>::max();
        if (control_point_count >
            (maximum - knot_count) / std::size_t(4)) {
            throw std::overflow_error(
                "3D array NURBS parameter count overflows size_t");
        }
        std::vector<point3<REAL>> control_points(control_point_count);
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
        return pack_parameters(
            control_points,
            std::vector<REAL>(control_point_count, REAL(1)),
            knots,
            degree);
    }

    [[nodiscard]] static std::valarray<REAL> pack_parameters(
        const std::vector<point3<REAL>>& control_points,
        const std::vector<REAL>& weights,
        const std::vector<REAL>& knots,
        std::size_t degree) {
        const std::size_t control_count = control_points.size();
        if (weights.size() != control_count) {
            throw std::invalid_argument(
                "NURBS weights and control points must have equal size");
        }
        const std::size_t required_knots =
            nurbs_knot_count(degree, control_count);
        if (knots.size() != required_knots) {
            throw std::invalid_argument(
                "NURBS knot count must equal control count + degree + 1");
        }
        if (control_count >
            (std::numeric_limits<std::size_t>::max() - required_knots) / 4) {
            throw std::overflow_error(
                "3D array NURBS parameter count overflows size_t");
        }

        std::valarray<REAL> result(
            REAL(0), control_count * std::size_t(4) + required_knots);
        for (std::size_t index = 0; index < control_count; ++index) {
            const std::size_t coordinate_offset = index * std::size_t(3);
            result[coordinate_offset] = control_points[index].x;
            result[coordinate_offset + 1] = control_points[index].y;
            result[coordinate_offset + 2] = control_points[index].z;
            result[control_count * std::size_t(3) + index] = weights[index];
        }
        const std::size_t knot_offset = control_count * std::size_t(4);
        for (std::size_t index = 0; index < required_knots; ++index) {
            result[knot_offset + index] = knots[index];
        }
        return result;
    }

    [[nodiscard]] layout checked_layout() const {
        if (degree_ == 0) {
            throw std::invalid_argument("NURBS degree must be positive");
        }
        if (parameters.size() <= degree_) {
            throw std::invalid_argument(
                "3D array NURBS parameter count is too small for its degree");
        }
        const std::size_t nondegree_count =
            parameters.size() - degree_ - std::size_t(1);
        if (nondegree_count % std::size_t(5) != 0) {
            throw std::invalid_argument(
                "3D array NURBS parameter count must equal 5*C + degree + 1");
        }
        const std::size_t control_count =
            nondegree_count / std::size_t(5);
        const std::size_t knot_count =
            nurbs_knot_count(degree_, control_count);
        const std::size_t weight_offset = control_count * std::size_t(3);
        const std::size_t knot_offset = control_count * std::size_t(4);
        if (knot_offset > parameters.size() ||
            knot_count != parameters.size() - knot_offset) {
            throw std::invalid_argument(
                "3D array NURBS parameter layout is inconsistent");
        }
        return {control_count, weight_offset, knot_offset, knot_count};
    }

    void validate_values(const layout& current) const {
        if (!(tolerance_ > REAL(0)) || !std::isfinite(tolerance_)) {
            throw std::invalid_argument(
                "NURBS tolerance must be finite and positive");
        }
        for (std::size_t index = 0;
             index < current.weight_offset; ++index) {
            if (!std::isfinite(parameters[index])) {
                throw std::invalid_argument(
                    "NURBS control points must be finite");
            }
        }
        for (std::size_t index = 0;
             index < current.control_count; ++index) {
            const REAL value = parameters[current.weight_offset + index];
            if (!(value > REAL(0)) || !std::isfinite(value)) {
                throw std::invalid_argument(
                    "NURBS weights must be finite and positive");
            }
        }
        for (std::size_t index = 0; index < current.knot_count; ++index) {
            const REAL value = parameters[current.knot_offset + index];
            if (!std::isfinite(value)) {
                throw std::invalid_argument("NURBS knots must be finite");
            }
            if (index > 0 &&
                value < parameters[current.knot_offset + index - 1]) {
                throw std::invalid_argument(
                    "NURBS knots must be nondecreasing");
            }
        }
        if (!(s_max_unchecked(current) > s_min_unchecked(current))) {
            throw std::invalid_argument(
                "NURBS active parameter domain must have positive length");
        }
    }

    [[nodiscard]] layout validated_layout() const {
        const layout current = checked_layout();
        validate_values(current);
        const point3<REAL> start = point_from_vector(
            rational_derivatives_at_unchecked(
                s_min_unchecked(current), 0, current)[0]);
        const point3<REAL> end = point_from_vector(
            rational_derivatives_at_unchecked(
                s_max_unchecked(current), 0, current)[0]);
        if (closed_ && distance(start, end) > closure_tolerance(current)) {
            throw std::invalid_argument(
                "closed NURBS endpoints must coincide");
        }
        return current;
    }

    [[nodiscard]] point3<REAL> control_point_unchecked(
        std::size_t index) const {
        const std::size_t offset = index * std::size_t(3);
        return {
            parameters[offset],
            parameters[offset + 1],
            parameters[offset + 2]};
    }

    [[nodiscard]] static point3<REAL> point_from_vector(
        const vector3<REAL>& value) noexcept {
        return {value.x, value.y, value.z};
    }

    [[nodiscard]] REAL knot_unchecked(
        const layout& current,
        std::size_t index) const {
        return parameters[current.knot_offset + index];
    }

    [[nodiscard]] REAL s_min_unchecked(const layout& current) const {
        return knot_unchecked(current, degree_);
    }

    [[nodiscard]] REAL s_max_unchecked(const layout& current) const {
        return knot_unchecked(current, current.control_count);
    }

    [[nodiscard]] REAL closure_tolerance(const layout& current) const noexcept {
        REAL scale = REAL(1);
        for (std::size_t index = 0;
             index < current.control_count; ++index) {
            const point3<REAL> point = control_point_unchecked(index);
            scale = std::max({
                scale,
                std::abs(point.x),
                std::abs(point.y),
                std::abs(point.z)});
        }
        return REAL(16) * tolerance_ * scale;
    }

    [[nodiscard]] REAL checked_parameter_unchecked(
        REAL s,
        const layout& current) const {
        if (!std::isfinite(s)) {
            throw std::out_of_range("NURBS parameter s must be finite");
        }
        const REAL lower = s_min_unchecked(current);
        const REAL upper = s_max_unchecked(current);
        if (s < lower - tolerance_ || s > upper + tolerance_) {
            throw std::out_of_range(
                "NURBS parameter s is outside the active knot domain");
        }
        return std::clamp(s, lower, upper);
    }

    [[nodiscard]] std::size_t find_span_unchecked(
        REAL s,
        const layout& current) const noexcept {
        const std::size_t n = current.control_count - 1;
        if (s >= knot_unchecked(current, n + 1)) {
            std::size_t span = n;
            while (span > degree_ &&
                   knot_unchecked(current, span) ==
                       knot_unchecked(current, span + 1)) {
                --span;
            }
            return span;
        }
        if (s <= knot_unchecked(current, degree_)) {
            std::size_t span = degree_;
            while (span < n &&
                   knot_unchecked(current, span) ==
                       knot_unchecked(current, span + 1)) {
                ++span;
            }
            return span;
        }

        std::size_t lower = degree_;
        std::size_t upper = n + 1;
        std::size_t middle = (lower + upper) / 2;
        while (s < knot_unchecked(current, middle) ||
               s >= knot_unchecked(current, middle + 1)) {
            if (s < knot_unchecked(current, middle)) {
                upper = middle;
            } else {
                lower = middle;
            }
            middle = (lower + upper) / 2;
        }
        return middle;
    }

    [[nodiscard]] std::vector<std::vector<REAL>>
    basis_function_derivatives_unchecked(
        std::size_t span,
        REAL s,
        std::size_t order,
        const layout& current) const {
        order = std::min(order, degree_);
        const std::size_t p = degree_;
        std::vector<std::vector<REAL>> ndu(
            p + 1, std::vector<REAL>(p + 1, REAL(0)));
        std::vector<REAL> left(p + 1, REAL(0));
        std::vector<REAL> right(p + 1, REAL(0));
        ndu[0][0] = REAL(1);

        for (std::size_t j = 1; j <= p; ++j) {
            left[j] = s - knot_unchecked(current, span + 1 - j);
            right[j] = knot_unchecked(current, span + j) - s;
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
                        ndu[static_cast<std::size_t>(pk + 1)]
                           [static_cast<std::size_t>(rk)];
                    derivative = work[destination][0] *
                        ndu[static_cast<std::size_t>(rk)]
                           [static_cast<std::size_t>(pk)];
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
                    derivative +=
                        work[destination][static_cast<std::size_t>(j)] *
                        ndu[static_cast<std::size_t>(rk + j)]
                           [static_cast<std::size_t>(pk)];
                }

                if (signed_r <= pk) {
                    work[destination][k] = -work[source][k - 1] /
                        ndu[static_cast<std::size_t>(pk + 1)][r];
                    derivative += work[destination][k] *
                        ndu[r][static_cast<std::size_t>(pk)];
                }
                derivatives[k][r] = derivative;
                std::swap(source, destination);
            }
        }

        REAL scale = static_cast<REAL>(p);
        for (std::size_t k = 1; k <= order; ++k) {
            for (std::size_t j = 0; j <= p; ++j) {
                derivatives[k][j] *= scale;
            }
            scale *= static_cast<REAL>(p - k);
        }
        return derivatives;
    }

    [[nodiscard]] std::array<vector3<REAL>, 4>
    rational_derivatives_at_unchecked(
        REAL s,
        std::size_t requested_order,
        const layout& current) const {
        requested_order = std::min<std::size_t>(requested_order, 3);
        const REAL parameter = checked_parameter_unchecked(s, current);
        const std::size_t span = find_span_unchecked(parameter, current);
        const std::size_t basis_order =
            std::min(requested_order, degree_);
        const auto basis_derivatives = basis_function_derivatives_unchecked(
            span, parameter, basis_order, current);

        std::array<vector3<REAL>, 4> numerator{};
        std::array<REAL, 4> weight_derivative{};
        for (std::size_t order = 0; order <= basis_order; ++order) {
            for (std::size_t local = 0; local <= degree_; ++local) {
                const std::size_t control_index =
                    span - degree_ + local;
                const REAL weight_value =
                    parameters[current.weight_offset + control_index];
                const REAL coefficient =
                    basis_derivatives[order][local] * weight_value;
                const point3<REAL> control =
                    control_point_unchecked(control_index);
                numerator[order] += coefficient * vector3<REAL>{
                    control.x, control.y, control.z};
                weight_derivative[order] += coefficient;
            }
        }

        if (std::abs(weight_derivative[0]) <= tolerance_) {
            throw std::domain_error(
                "NURBS homogeneous weight is near zero");
        }

        std::array<vector3<REAL>, 4> result{};
        result[0] = numerator[0] / weight_derivative[0];
        for (std::size_t order = 1; order <= requested_order; ++order) {
            vector3<REAL> value = numerator[order];
            REAL binomial = REAL(1);
            for (std::size_t weight_order = 1;
                 weight_order <= order; ++weight_order) {
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

    std::size_t degree_;
    REAL tolerance_;
    bool closed_ = false;
};

template <std::floating_point REAL>
nurbs_spline3<REAL>::nurbs_spline3(
    const nurbs_arr_spline3<REAL>& spline)
    : nurbs_spline3(
          spline.get_control_points(),
          spline.get_weights(),
          spline.get_knots(),
          spline.degree(),
          spline.is_closed(),
          spline.tolerance()) {}

} // namespace nurbspath
