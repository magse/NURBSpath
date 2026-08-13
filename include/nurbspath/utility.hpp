#pragma once

#include "nurbspath/config.hpp"
#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace nurbspath {

/**
 * @brief Controls iterative and sampled numerical algorithms.
 *
 * `residual_tolerance` uses function-result units and `parameter_tolerance`
 * uses parameter units. Increase `sample_count` for oscillatory functions.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct numerical_settings {
    REAL residual_tolerance = REAL(1e-8); ///< Accepted absolute function residual.
    REAL parameter_tolerance = REAL(1e-10); ///< Accepted parameter interval width.
    std::size_t max_iterations = 100; ///< Maximum refinement iterations.
    std::size_t sample_count = 256; ///< Number of initial uniform intervals.

    /**
     * @brief Validate all numerical controls.
     * @throws std::invalid_argument When a tolerance/count is not positive or sample_count is below two.
     */
    void validate() const {
        if (!(residual_tolerance > REAL(0))) {
            throw std::invalid_argument("residual_tolerance must be positive");
        }
        if (!(parameter_tolerance > REAL(0))) {
            throw std::invalid_argument("parameter_tolerance must be positive");
        }
        if (max_iterations == 0) {
            throw std::invalid_argument("max_iterations must be positive");
        }
        if (sample_count < 2) {
            throw std::invalid_argument("sample_count must be at least two");
        }
    }
};

template <std::floating_point REAL>
/**
 * @brief Square a scalar.
 * @tparam REAL Floating-point scalar type.
 * @param value Scalar value.
 * @return `value * value`.
 */
[[nodiscard]] constexpr REAL square(REAL value) noexcept {
    return value * value;
}

template <std::floating_point REAL>
/**
 * @brief Compare two scalar values with absolute tolerance.
 * @tparam REAL Floating-point scalar type.
 * @param first First scalar.
 * @param second Second scalar.
 * @param tolerance Maximum accepted absolute difference.
 * @return True when absolute difference is at most tolerance.
 */
[[nodiscard]] bool approximately_equal(
    REAL first,
    REAL second,
    REAL tolerance) noexcept {
    return std::abs(first - second) <= tolerance;
}

/**
 * @brief Solve a square linear system using scaled partial pivoting.
 * @tparam REAL Floating-point scalar type.
 * @param matrix Square coefficient matrix A, copied for elimination.
 * @param right_hand_side Right-hand-side vector b, copied for elimination.
 * @param pivot_tolerance Relative pivot rejection threshold.
 * @return Solution vector x for `A*x=b`.
 * @throws std::invalid_argument When matrix dimensions are invalid.
 * @throws std::domain_error When the matrix is singular or ill-conditioned.
 */
template <std::floating_point REAL>
[[nodiscard]] std::vector<REAL> solve_linear_system(
    std::vector<std::vector<REAL>> matrix,
    std::vector<REAL> right_hand_side,
    REAL pivot_tolerance = REAL(64) * std::numeric_limits<REAL>::epsilon()) {
    const std::size_t size = matrix.size();
    if (size == 0 || right_hand_side.size() != size) {
        throw std::invalid_argument("linear system must be non-empty and square");
    }
    for (const auto& row : matrix) {
        if (row.size() != size) {
            throw std::invalid_argument("linear system matrix must be square");
        }
    }

    // A scale for every row avoids rejecting a small but otherwise well-scaled
    // system, and makes pivot selection more stable for mixed magnitudes.
    std::vector<REAL> row_scales(size, REAL(0));
    for (std::size_t row = 0; row < size; ++row) {
        for (REAL value : matrix[row]) {
            row_scales[row] = std::max(row_scales[row], std::abs(value));
        }
        if (row_scales[row] == REAL(0)) {
            throw std::domain_error("linear system is singular");
        }
    }

    for (std::size_t column = 0; column < size; ++column) {
        std::size_t pivot_row = column;
        REAL best_scaled_pivot = REAL(-1);
        for (std::size_t row = column; row < size; ++row) {
            const REAL scaled = std::abs(matrix[row][column]) / row_scales[row];
            if (scaled > best_scaled_pivot) {
                best_scaled_pivot = scaled;
                pivot_row = row;
            }
        }

        if (std::abs(matrix[pivot_row][column]) <=
            pivot_tolerance * row_scales[pivot_row]) {
            throw std::domain_error("linear system is singular or ill-conditioned");
        }

        if (pivot_row != column) {
            std::swap(matrix[pivot_row], matrix[column]);
            std::swap(right_hand_side[pivot_row], right_hand_side[column]);
            std::swap(row_scales[pivot_row], row_scales[column]);
        }

        for (std::size_t row = column + 1; row < size; ++row) {
            const REAL factor = matrix[row][column] / matrix[column][column];
            matrix[row][column] = REAL(0);
            for (std::size_t entry = column + 1; entry < size; ++entry) {
                matrix[row][entry] -= factor * matrix[column][entry];
            }
            right_hand_side[row] -= factor * right_hand_side[column];
        }
    }

    std::vector<REAL> solution(size, REAL(0));
    for (std::size_t reverse = size; reverse-- > 0;) {
        REAL remainder = right_hand_side[reverse];
        for (std::size_t column = reverse + 1; column < size; ++column) {
            remainder -= matrix[reverse][column] * solution[column];
        }
        solution[reverse] = remainder / matrix[reverse][reverse];
    }
    return solution;
}

/**
 * @brief Refine a sign-changing root interval by bisection.
 * @tparam REAL Floating-point scalar type.
 * @tparam FUNCTION Callable accepting REAL and returning a scalar residual.
 * @param function Scalar residual function.
 * @param lower First interval endpoint.
 * @param upper Second interval endpoint; endpoints may be reversed.
 * @param settings Numerical tolerances and iteration limit.
 * @return Refined root parameter.
 * @throws std::invalid_argument When settings are invalid or endpoints do not bracket a root.
 */
template <std::floating_point REAL, typename FUNCTION>
[[nodiscard]] REAL bisect_root(
    FUNCTION&& function,
    REAL lower,
    REAL upper,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    if (upper < lower) {
        std::swap(lower, upper);
    }

    REAL lower_value = function(lower);
    REAL upper_value = function(upper);
    if (std::abs(lower_value) <= settings.residual_tolerance) {
        return lower;
    }
    if (std::abs(upper_value) <= settings.residual_tolerance) {
        return upper;
    }
    if (std::signbit(lower_value) == std::signbit(upper_value)) {
        throw std::invalid_argument("bisect_root requires a sign-changing interval");
    }

    for (std::size_t iteration = 0; iteration < settings.max_iterations; ++iteration) {
        const REAL middle = lower + (upper - lower) / REAL(2);
        const REAL middle_value = function(middle);
        if (std::abs(middle_value) <= settings.residual_tolerance ||
            upper - lower <= settings.parameter_tolerance) {
            return middle;
        }

        if (std::signbit(lower_value) != std::signbit(middle_value)) {
            upper = middle;
            upper_value = middle_value;
        } else {
            lower = middle;
            lower_value = middle_value;
        }
    }
    return lower + (upper - lower) / REAL(2);
}

/**
 * @brief Minimize a scalar function on a closed interval by golden-section search.
 * @tparam REAL Floating-point scalar type.
 * @tparam FUNCTION Callable accepting REAL and returning an ordered scalar value.
 * @param function Objective function.
 * @param lower First interval endpoint.
 * @param upper Second interval endpoint; endpoints may be reversed.
 * @param settings Parameter tolerance and iteration limit.
 * @return Parameter at the refined sampled minimum.
 */
template <std::floating_point REAL, typename FUNCTION>
[[nodiscard]] REAL golden_section_minimize(
    FUNCTION&& function,
    REAL lower,
    REAL upper,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    if (upper < lower) {
        std::swap(lower, upper);
    }

    constexpr long double inverse_phi_ld = 0.6180339887498948482045868343656381L;
    const REAL inverse_phi = static_cast<REAL>(inverse_phi_ld);
    REAL left_probe = upper - inverse_phi * (upper - lower);
    REAL right_probe = lower + inverse_phi * (upper - lower);
    REAL left_value = function(left_probe);
    REAL right_value = function(right_probe);

    for (std::size_t iteration = 0;
         iteration < settings.max_iterations &&
         upper - lower > settings.parameter_tolerance;
         ++iteration) {
        if (left_value <= right_value) {
            upper = right_probe;
            right_probe = left_probe;
            right_value = left_value;
            left_probe = upper - inverse_phi * (upper - lower);
            left_value = function(left_probe);
        } else {
            lower = left_probe;
            left_probe = right_probe;
            left_value = right_value;
            right_probe = lower + inverse_phi * (upper - lower);
            right_value = function(right_probe);
        }
    }
    return lower + (upper - lower) / REAL(2);
}

/**
 * @brief Discover and refine roots on a bounded sampled interval.
 *
 * Sign changes use bisection and local minima of absolute residual capture
 * tangent/even-multiplicity contacts. Coverage depends on `sample_count`.
 *
 * @tparam REAL Floating-point scalar type.
 * @tparam FUNCTION Callable accepting REAL and returning a scalar residual.
 * @param function Scalar residual function.
 * @param lower Strict lower search bound.
 * @param upper Strict upper search bound.
 * @param settings Sampling and refinement controls.
 * @return Sorted, tolerance-deduplicated root parameters.
 * @throws std::invalid_argument When settings or interval are invalid.
 * @throws std::domain_error When function produces a non-finite value.
 */
template <std::floating_point REAL, typename FUNCTION>
[[nodiscard]] std::vector<REAL> find_roots(
    FUNCTION&& function,
    REAL lower,
    REAL upper,
    const numerical_settings<REAL>& settings = {}) {
    settings.validate();
    if (!(upper > lower)) {
        throw std::invalid_argument("root search interval must have positive length");
    }

    const std::size_t sample_count = settings.sample_count;
    std::vector<REAL> parameters(sample_count + 1);
    std::vector<REAL> values(sample_count + 1);
    for (std::size_t index = 0; index <= sample_count; ++index) {
        const REAL fraction = static_cast<REAL>(index) / static_cast<REAL>(sample_count);
        parameters[index] = lower + fraction * (upper - lower);
        values[index] = function(parameters[index]);
        if (!std::isfinite(values[index])) {
            throw std::domain_error("root function produced a non-finite value");
        }
    }

    std::vector<REAL> roots;
    const REAL merge_tolerance = std::max(
        settings.parameter_tolerance * REAL(4),
        (upper - lower) * REAL(16) * std::numeric_limits<REAL>::epsilon());
    const auto add_root = [&](REAL candidate) {
        candidate = std::clamp(candidate, lower, upper);
        for (REAL existing : roots) {
            if (std::abs(existing - candidate) <= merge_tolerance) {
                return;
            }
        }
        roots.push_back(candidate);
    };

    for (std::size_t index = 0; index < sample_count; ++index) {
        if (
            std::signbit(values[index]) != std::signbit(values[index + 1])) {
            add_root(bisect_root<REAL>(
                function, parameters[index], parameters[index + 1], settings));
        }
    }

    // Collapse a contiguous band of within-tolerance samples to its smallest
    // residual. A very flat tangent must represent one contact rather than one
    // contact per sample. Bands adjacent to a sign change were already refined.
    std::size_t band_start = 0;
    while (band_start <= sample_count) {
        if (std::abs(values[band_start]) > settings.residual_tolerance) {
            ++band_start;
            continue;
        }
        std::size_t band_end = band_start;
        std::size_t best_index = band_start;
        while (band_end + 1 <= sample_count &&
               std::abs(values[band_end + 1]) <= settings.residual_tolerance) {
            ++band_end;
            if (std::abs(values[band_end]) < std::abs(values[best_index])) {
                best_index = band_end;
            }
        }

        bool adjacent_to_sign_change = false;
        const std::size_t first_interval = band_start == 0 ? 0 : band_start - 1;
        const std::size_t last_interval = std::min(band_end, sample_count - 1);
        for (std::size_t interval = first_interval;
             interval <= last_interval && sample_count > 0;
             ++interval) {
            if (std::signbit(values[interval]) !=
                std::signbit(values[interval + 1])) {
                adjacent_to_sign_change = true;
                break;
            }
        }
        if (!adjacent_to_sign_change) {
            REAL candidate = parameters[best_index];
            const std::size_t refinement_start = band_start == 0 ? 0 : band_start - 1;
            const std::size_t refinement_end =
                std::min(band_end + 1, sample_count);
            if (refinement_start < refinement_end) {
                const REAL refined = golden_section_minimize<REAL>(
                    [&](REAL parameter) { return std::abs(function(parameter)); },
                    parameters[refinement_start],
                    parameters[refinement_end],
                    settings);
                if (std::abs(function(refined)) < std::abs(function(candidate))) {
                    candidate = refined;
                }
            }
            add_root(candidate);
        }
        band_start = band_end + 1;
    }

    // Tangent contacts appear as a local minimum of the absolute residual.
    for (std::size_t index = 1; index < sample_count; ++index) {
        const REAL previous = std::abs(values[index - 1]);
        const REAL current = std::abs(values[index]);
        const REAL next = std::abs(values[index + 1]);
        const bool sign_change_before =
            std::signbit(values[index - 1]) != std::signbit(values[index]);
        const bool sign_change_after =
            std::signbit(values[index]) != std::signbit(values[index + 1]);
        // A sampled zero or a neighboring sign change was already handled by
        // the loop above. Only sign-preserving minima need the tangency pass.
        if (current > settings.residual_tolerance &&
            !sign_change_before && !sign_change_after &&
            current <= previous && current <= next) {
            const REAL candidate = golden_section_minimize<REAL>(
                [&](REAL parameter) { return std::abs(function(parameter)); },
                parameters[index - 1],
                parameters[index + 1],
                settings);
            if (std::abs(function(candidate)) <= settings.residual_tolerance) {
                add_root(candidate);
            }
        }
    }

    std::sort(roots.begin(), roots.end());
    return roots;
}

} // namespace nurbspath
