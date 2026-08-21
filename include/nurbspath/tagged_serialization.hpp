#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/nurbs_spline3.hpp"
#include "nurbspath/point3.hpp"
#include "nurbspath/serialization.hpp"
#include "nurbspath/sphere3.hpp"
#include "nurbspath/vector3.hpp"

#include <concepts>
#include <cstddef>
#include <istream>
#include <memory>
#include <optional>
#include <utility>
#include <variant>

namespace nurbspath {

/**
 * @brief Type-safe pointer alternatives supported by tagged 3D input.
 * @tparam REAL Floating-point scalar type used to decode numeric fields.
 */
template <std::floating_point REAL>
using entity3_ptr = std::variant<
    std::shared_ptr<point3<REAL>>,
    std::shared_ptr<vector3<REAL>>,
    std::shared_ptr<sphere3<REAL>>,
    std::shared_ptr<nurbs_spline3<REAL>>>;

/**
 * @brief Application tag and dynamically selected concrete 3D entity.
 * @tparam REAL Floating-point scalar type used by the stored entity.
 */
template <std::floating_point REAL>
struct tagged_entity3 {
    std::size_t tag = 0; ///< Application-defined tag read from the row prefix.
    /// Entity pointer; successful reads always select a non-null shared owner.
    entity3_ptr<REAL> entity;
};

namespace detail {

/** @cond */

template <std::floating_point REAL, typename ENTITY>
[[nodiscard]] std::optional<tagged_entity3<REAL>> wrap_tagged_entity3(
    std::optional<tagged_read_result<ENTITY>> decoded) {
    if (!decoded) {
        return std::nullopt;
    }
    return tagged_entity3<REAL>{
        decoded->tag, entity3_ptr<REAL>{std::move(decoded->entity)}};
}

/** @endcond */

} // namespace detail

/**
 * @brief Read one version-1 tagged row and allocate its concrete 3D entity.
 *
 * The case-sensitive type token dispatches `point3`, `vector3`, `sphere3`, or
 * `spline3`. When a row is present, exactly that physical row is consumed. The
 * complete grammar and recovery rules are documented in `DATA.md`.
 *
 * @tparam REAL Floating-point scalar type used to decode numeric fields.
 * @param input Source text stream positioned at the start of a row.
 * @return Tag and entity variant selecting a non-null shared owner after
 * success, or `std::nullopt` at EOF, another read failure, or after a malformed
 * row. Unknown types, invalid definitions, and trailing fields set `failbit`.
 * @throws std::bad_alloc When buffering the row or allocating an entity fails.
 * @throws std::ios_base::failure When enabled by the stream exception mask.
 */
template <std::floating_point REAL>
[[nodiscard]] std::optional<tagged_entity3<REAL>> tag_read(
    std::istream& input) {
    const auto record = detail::read_tagged_text_record(input);
    if (!record) {
        return std::nullopt;
    }

    if (record->entity_type == "point3") {
        return detail::wrap_tagged_entity3<REAL>(
            detail::decode_tagged_point3<REAL>(*record, input));
    }
    if (record->entity_type == "vector3") {
        return detail::wrap_tagged_entity3<REAL>(
            detail::decode_tagged_vector3<REAL>(*record, input));
    }
    if (record->entity_type == "sphere3") {
        return detail::wrap_tagged_entity3<REAL>(
            detail::decode_tagged_sphere3<REAL>(*record, input));
    }
    if (record->entity_type == "spline3") {
        return detail::wrap_tagged_entity3<REAL>(
            detail::decode_tagged_spline3<REAL>(*record, input));
    }

    detail::mark_tagged_read_failure(input);
    return std::nullopt;
}

} // namespace nurbspath
