#pragma once

#include "nurbspath/config.hpp"
#include "nurbspath/nurbs_spline3.hpp"
#include "nurbspath/plane3.hpp"
#include "nurbspath/point3.hpp"
#include "nurbspath/projection.hpp"
#include "nurbspath/ray3.hpp"
#include "nurbspath/sphere3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <locale>
#include <numbers>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace nurbspath {

/**
 * @brief Flat viewport definition for the two-dimensional SVG canvas.
 *
 * The 2D world is mapped directly to the SVG plane with positive X to the
 * right and positive Y upward. A uniform scale preserves circles when viewport
 * and image aspect ratios differ; unused space is letterboxed.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class svg_view2 {
public:
    /**
     * @brief Construct a flat 2D viewport.
     * @param center 2D world point placed at the image center.
     * @param viewport_width Minimum visible horizontal extent in 2D world units.
     * @param viewport_height Minimum visible vertical extent in 2D world units.
     * @param image_width SVG viewBox width.
     * @param image_height SVG viewBox height.
     * @throws std::invalid_argument When center or a dimension is invalid.
     */
    svg_view2(
        const point2<REAL>& center,
        REAL viewport_width,
        REAL viewport_height,
        std::size_t image_width = 800,
        std::size_t image_height = 600)
        : center_(center),
          viewport_width_(viewport_width),
          viewport_height_(viewport_height),
          image_width_(image_width),
          image_height_(image_height) {
        validate();
    }

    /** @brief Get the 2D world point at image center. @return Center reference. */
    [[nodiscard]] const point2<REAL>& center() const noexcept { return center_; }

    /** @brief Get the minimum visible world width. @return Positive width. */
    [[nodiscard]] REAL viewport_width() const noexcept { return viewport_width_; }

    /** @brief Get the minimum visible world height. @return Positive height. */
    [[nodiscard]] REAL viewport_height() const noexcept { return viewport_height_; }

    /** @brief Get the SVG viewBox width. @return Positive width. */
    [[nodiscard]] std::size_t image_width() const noexcept { return image_width_; }

    /** @brief Get the SVG viewBox height. @return Positive height. */
    [[nodiscard]] std::size_t image_height() const noexcept { return image_height_; }

private:
    void validate() const {
        if (!std::isfinite(center_.x()) || !std::isfinite(center_.y())) {
            throw std::invalid_argument("2D SVG view center must be finite");
        }
        if (!(viewport_width_ > REAL(0)) ||
            !(viewport_height_ > REAL(0)) ||
            !std::isfinite(viewport_width_) ||
            !std::isfinite(viewport_height_)) {
            throw std::invalid_argument(
                "2D SVG viewport dimensions must be finite and positive");
        }
        if (image_width_ == 0 || image_height_ == 0) {
            throw std::invalid_argument("2D SVG image dimensions must be positive");
        }
    }

    point2<REAL> center_;
    REAL viewport_width_;
    REAL viewport_height_;
    std::size_t image_width_;
    std::size_t image_height_;
};

/**
 * @brief Styling and tessellation controls for the flat 2D SVG canvas.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct svg_graphics_options2 {
    /// SVG line width in viewBox units, normally equivalent to output pixels.
    REAL line_width = REAL(1.5);

    /// Number of solid straight elements used to approximate a NURBS path.
    std::size_t spline_segment_count = 128;

    /**
     * @brief Validate all flat-canvas graphics controls.
     * @throws std::invalid_argument When line width or segment count is invalid.
     */
    void validate() const {
        if (!(line_width > REAL(0)) || !std::isfinite(line_width)) {
            throw std::invalid_argument(
                "2D SVG line width must be finite and positive");
        }
        if (spline_segment_count == 0) {
            throw std::invalid_argument(
                "2D SVG spline segment count must be positive");
        }
    }
};

/** @brief Camera projections supported by the SVG diagnostic renderer. */
enum class svg_projection {
    orthographic, ///< Parallel projection with world-space viewport extents.
    perspective ///< Pinhole projection with vertical field angle.
};

/**
 * @brief Camera and viewport definition for a three-dimensional SVG view.
 *
 * Roll is resolved automatically by preferring world +z, or world +y when the
 * viewing direction is nearly parallel to +z.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class svg_view3 {
public:
    /**
     * @brief Construct an orthographic camera.
     * @param eyepoint World-space camera location controlling view direction.
     * @param lookatpoint World-space point at the image center.
     * @param viewport_width Minimum visible horizontal world extent.
     * @param viewport_height Minimum visible vertical world extent.
     * @param image_width SVG viewBox width.
     * @param image_height SVG viewBox height.
     * @return Validated orthographic view definition.
     * @throws std::invalid_argument When camera points or dimensions are invalid.
     */
    [[nodiscard]] static svg_view3 orthographic(
        const point3<REAL>& eyepoint,
        const point3<REAL>& lookatpoint,
        REAL viewport_width,
        REAL viewport_height,
        std::size_t image_width = 800,
        std::size_t image_height = 600) {
        return svg_view3(
            eyepoint,
            lookatpoint,
            svg_projection::orthographic,
            viewport_width,
            viewport_height,
            REAL(0),
            REAL(0),
            image_width,
            image_height);
    }

    /**
     * @brief Construct a perspective camera.
     * @param eyepoint World-space pinhole location.
     * @param lookatpoint World-space point at the image center.
     * @param vertical_view_angle Vertical field angle in radians in `(0,pi)`.
     * @param image_width SVG viewBox width.
     * @param image_height SVG viewBox height.
     * @param near_distance Positive near clipping distance in world units.
     * @return Validated perspective view definition.
     * @throws std::invalid_argument When camera points, angle, clip, or dimensions are invalid.
     */
    [[nodiscard]] static svg_view3 perspective(
        const point3<REAL>& eyepoint,
        const point3<REAL>& lookatpoint,
        REAL vertical_view_angle,
        std::size_t image_width = 800,
        std::size_t image_height = 600,
        REAL near_distance = REAL(1e-4)) {
        return svg_view3(
            eyepoint,
            lookatpoint,
            svg_projection::perspective,
            REAL(0),
            REAL(0),
            vertical_view_angle,
            near_distance,
            image_width,
            image_height);
    }

    /** @brief Get camera location. @return Constant eyepoint reference. */
    [[nodiscard]] const point3<REAL>& eyepoint() const noexcept { return eyepoint_; }
    /** @brief Get center-of-view target. @return Constant look-at reference. */
    [[nodiscard]] const point3<REAL>& lookatpoint() const noexcept { return lookatpoint_; }
    /** @brief Get projection mode. @return Orthographic or perspective mode. */
    [[nodiscard]] svg_projection projection() const noexcept { return projection_; }
    /** @brief Get orthographic world width. @return Width, or zero for perspective. */
    [[nodiscard]] REAL viewport_width() const noexcept { return viewport_width_; }
    /** @brief Get orthographic world height. @return Height, or zero for perspective. */
    [[nodiscard]] REAL viewport_height() const noexcept { return viewport_height_; }
    /** @brief Get vertical perspective field angle. @return Radians, or zero for orthographic. */
    [[nodiscard]] REAL vertical_view_angle() const noexcept {
        return vertical_view_angle_;
    }
    /** @brief Get perspective near distance. @return Distance, or zero for orthographic. */
    [[nodiscard]] REAL near_distance() const noexcept { return near_distance_; }
    /** @brief Get SVG viewBox width. @return Positive width. */
    [[nodiscard]] std::size_t image_width() const noexcept { return image_width_; }
    /** @brief Get SVG viewBox height. @return Positive height. */
    [[nodiscard]] std::size_t image_height() const noexcept { return image_height_; }

private:
    svg_view3(
        const point3<REAL>& eyepoint,
        const point3<REAL>& lookatpoint,
        svg_projection projection,
        REAL viewport_width,
        REAL viewport_height,
        REAL vertical_view_angle,
        REAL near_distance,
        std::size_t image_width,
        std::size_t image_height)
        : eyepoint_(eyepoint),
          lookatpoint_(lookatpoint),
          projection_(projection),
          viewport_width_(viewport_width),
          viewport_height_(viewport_height),
          vertical_view_angle_(vertical_view_angle),
          near_distance_(near_distance),
          image_width_(image_width),
          image_height_(image_height) {
        validate();
    }

    void validate() const {
        const auto finite_point = [](const point3<REAL>& point) {
            return std::isfinite(point.x()) &&
                   std::isfinite(point.y()) &&
                   std::isfinite(point.z());
        };
        if (!finite_point(eyepoint_) || !finite_point(lookatpoint_)) {
            throw std::invalid_argument("SVG camera points must be finite");
        }
        if ((lookatpoint_ - eyepoint_).is_near_zero()) {
            throw std::invalid_argument("SVG eyepoint and lookatpoint must differ");
        }
        if (image_width_ == 0 || image_height_ == 0) {
            throw std::invalid_argument("SVG image dimensions must be positive");
        }

        if (projection_ == svg_projection::orthographic) {
            if (!(viewport_width_ > REAL(0)) ||
                !(viewport_height_ > REAL(0)) ||
                !std::isfinite(viewport_width_) ||
                !std::isfinite(viewport_height_)) {
                throw std::invalid_argument(
                    "orthographic viewport dimensions must be finite and positive");
            }
        } else {
            if (!(vertical_view_angle_ > REAL(0)) ||
                !(vertical_view_angle_ < std::numbers::pi_v<REAL>) ||
                !std::isfinite(vertical_view_angle_)) {
                throw std::invalid_argument(
                    "perspective vertical view angle must lie between zero and pi");
            }
            if (!(near_distance_ > REAL(0)) || !std::isfinite(near_distance_)) {
                throw std::invalid_argument(
                    "perspective near distance must be finite and positive");
            }
        }
    }

    point3<REAL> eyepoint_;
    point3<REAL> lookatpoint_;
    svg_projection projection_;
    REAL viewport_width_;
    REAL viewport_height_;
    REAL vertical_view_angle_;
    REAL near_distance_;
    std::size_t image_width_;
    std::size_t image_height_;
};

/**
 * @brief Styling and tessellation controls for SVG diagnostics.
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
struct svg_graphics_options {
    /// SVG line width in viewBox units, normally equivalent to output pixels.
    REAL line_width = REAL(1.5);

    /// Number of straight elements used to approximate a complete NURBS path.
    std::size_t spline_segment_count = 128;

    /// Number of straight elements used for the sphere outline and equator.
    std::size_t sphere_segment_count = 96;

    /// Side length of the centered plane marker square, in world units.
    /// The default is one quarter of the unit normal marker length.
    REAL plane_square_side_length = REAL(0.25);

    /// Length of the positive-u and positive-v plane direction markers.
    REAL plane_parameter_axis_length = REAL(0.5);

    /**
     * @brief Validate all graphics controls.
     * @throws std::invalid_argument When a width, length, or segment count is invalid.
     */
    void validate() const {
        if (!(line_width > REAL(0)) || !std::isfinite(line_width)) {
            throw std::invalid_argument("SVG line width must be finite and positive");
        }
        if (spline_segment_count == 0) {
            throw std::invalid_argument("SVG spline segment count must be positive");
        }
        if (sphere_segment_count < 12) {
            throw std::invalid_argument("SVG sphere segment count must be at least twelve");
        }
        if (!(plane_square_side_length > REAL(0)) ||
            !std::isfinite(plane_square_side_length)) {
            throw std::invalid_argument(
                "SVG plane square side length must be finite and positive");
        }
        if (!(plane_parameter_axis_length > REAL(0)) ||
            !std::isfinite(plane_parameter_axis_length)) {
            throw std::invalid_argument(
                "SVG plane parameter-axis length must be finite and positive");
        }
    }
};

/** @cond internal */
namespace detail {

/// Camera basis used internally by the SVG projection implementation.
template <std::floating_point REAL>
struct svg_camera_frame3 {
    vector3<REAL> forward;
    vector3<REAL> right;
    vector3<REAL> up;
    REAL look_distance;
};

/// Camera-space point used internally before projection to the SVG viewBox.
template <std::floating_point REAL>
struct svg_camera_point3 {
    REAL x;
    REAL y;
    REAL z;
};

/// Two-dimensional projected point used internally by SVG clipping.
template <std::floating_point REAL>
struct svg_screen_point3 {
    REAL x;
    REAL y;
};

/// Screen-space point used internally by the flat 2D SVG canvas.
template <std::floating_point REAL>
struct svg_screen_point2 {
    REAL x;
    REAL y;
};

/// Stroke patterns used internally to distinguish projected 2D geometry.
enum class svg_line_style {
    solid,
    dashed,
    dotted,
    dash_dotted
};

/// Exact native entity set accepted by the flat 2D SVG convenience overload.
template <typename ENTITY, typename REAL>
concept svg_canvas_entity2 =
    std::same_as<ENTITY, point2<REAL>> ||
    std::same_as<ENTITY, ray2<REAL>> ||
    std::same_as<ENTITY, circle2<REAL>> ||
    std::same_as<ENTITY, nurbs_spline2<REAL>>;

} // namespace detail
/** @endcond */

/**
 * @brief Accumulate native 2D geometry and emit a flat SVG document.
 *
 * This canvas accepts only `point2`, `ray2`, `circle2`, and `nurbs_spline2`.
 * It performs no `plane3` embedding and has no overload for any 3D entity or
 * free vector. All entity and construction lines are solid. Every canvas starts
 * with solid unit X and Y reference axes at the 2D origin.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class svg_canvas2 {
public:
    /**
     * @brief Construct an empty flat canvas containing the 2D XY reference.
     * @param view Validated flat viewport definition.
     * @param options Styling and spline tessellation controls.
     * @throws std::invalid_argument When options are invalid.
     */
    explicit svg_canvas2(
        const svg_view2<REAL>& view,
        const svg_graphics_options2<REAL>& options = {})
        : view_(view), options_(options) {
        options_.validate();
        elements_.imbue(std::locale::classic());
        elements_ << std::setprecision(std::numeric_limits<REAL>::max_digits10);
        draw_coordinate_system();
    }

    /**
     * @brief Add a native 2D point as a black dot.
     * @param point Point in the independent 2D world.
     */
    void add(const point2<REAL>& point) {
        const auto screen = world_to_screen(point);
        if (!screen) {
            return;
        }
        elements_ << "<circle class=\"nurbspath_canvas_point2\" cx=\""
                  << screen->x << "\" cy=\"" << screen->y
                  << "\" r=\"" << options_.line_width
                  << "\" fill=\"#000000\" stroke=\"none\"/>\n";
    }

    /**
     * @brief Add a native 2D ray using solid pale-red lines.
     * @param ray Ray in the independent 2D world.
     */
    void add(const ray2<REAL>& ray) {
        const auto screen_line = projected_infinite_line(ray);
        if (screen_line) {
            append_screen_line(
                "nurbspath_canvas_ray2",
                screen_line->first,
                screen_line->second,
                "#e7a0a0",
                REAL(0.9));
        }
        append_world_segment(
            "nurbspath_canvas_ray2_positive_direction",
            ray.origin(),
            ray.origin() + ray.direction().normalized(REAL(0)),
            "#d65c5c",
            REAL(1));
        append_location_dot(
            "nurbspath_canvas_ray2_origin", ray.origin(), "#d65c5c");
    }

    /**
     * @brief Add a native 2D circle and center using solid pale-blue lines.
     * @param circle Circle in the independent 2D world.
     */
    void add(const circle2<REAL>& circle) {
        const auto center = world_to_screen(circle.center());
        if (center) {
            elements_ << "<circle class=\"nurbspath_canvas_circle2\" cx=\""
                      << center->x << "\" cy=\"" << center->y
                      << "\" r=\"" << circle.radius() * scale()
                      << "\" fill=\"none\" stroke=\"#8fc5e8\" "
                      << "stroke-opacity=\"1\"/>\n";
        }
        append_location_dot(
            "nurbspath_canvas_circle2_center", circle.center(), "#4f8fcf");
    }

    /**
     * @brief Add a native 2D spline with solid curve and control lines.
     * @param spline NURBS spline in the independent 2D world.
     * @throws std::domain_error When the spline repeats without a finite limit.
     */
    void add(const nurbs_spline2<REAL>& spline) {
        if (spline.period_count() == 0) {
            throw std::domain_error(
                "SVG rendering requires a finite 2D spline domain");
        }
        const auto& control_points = spline.control_points();
        for (std::size_t index = 1; index < control_points.size(); ++index) {
            append_world_segment(
                "nurbspath_canvas_spline2_control_polygon",
                control_points[index - 1],
                control_points[index],
                "#b8a6cc",
                REAL(0.6));
        }
        for (const point2<REAL>& control_point : control_points) {
            append_location_dot(
                "nurbspath_canvas_spline2_control_point",
                control_point,
                "#80649b",
                REAL(1.4));
        }

        point2<REAL> previous = spline.evaluate(spline.s_min());
        for (std::size_t index = 1;
             index <= options_.spline_segment_count;
             ++index) {
            const REAL fraction = static_cast<REAL>(index) /
                                  static_cast<REAL>(options_.spline_segment_count);
            const REAL s = spline.s_min() +
                fraction * (spline.s_max() - spline.s_min());
            const point2<REAL> current = spline.evaluate(s);
            append_world_segment(
                "nurbspath_canvas_spline2",
                previous,
                current,
                "#000000",
                REAL(1));
            previous = current;
        }

        append_location_dot(
            "nurbspath_canvas_spline2_endpoint",
            spline.evaluate(spline.s_min()),
            "#000000",
            REAL(1.9));
        append_location_dot(
            "nurbspath_canvas_spline2_endpoint",
            spline.evaluate(spline.s_max()),
            "#000000",
            REAL(1.9));
    }

    /** @brief Serialize the flat canvas. @return Self-contained SVG XML string. */
    [[nodiscard]] std::string svg() const {
        std::ostringstream output;
        output.imbue(std::locale::classic());
        output << std::setprecision(std::numeric_limits<REAL>::max_digits10);
        output << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
               << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\""
               << view_.image_width() << "\" height=\"" << view_.image_height()
               << "\" viewBox=\"0 0 " << view_.image_width() << ' '
               << view_.image_height() << "\">\n"
               << "<defs><clipPath id=\"nurbspath_canvas2_viewport\"><rect "
               << "x=\"0\" y=\"0\" width=\"" << view_.image_width()
               << "\" height=\"" << view_.image_height()
               << "\"/></clipPath></defs>\n"
               << "<g clip-path=\"url(#nurbspath_canvas2_viewport)\" "
               << "fill=\"none\" stroke-width=\"" << options_.line_width
               << "\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n"
               << elements_.str()
               << "</g>\n</svg>\n";
        return output.str();
    }

    /**
     * @brief Stream the complete flat SVG document.
     * @param output Destination stream.
     */
    void write(std::ostream& output) const { output << svg(); }

private:
    using screen_point = detail::svg_screen_point2<REAL>;

    [[nodiscard]] REAL scale() const noexcept {
        return std::min(
            static_cast<REAL>(view_.image_width()) / view_.viewport_width(),
            static_cast<REAL>(view_.image_height()) / view_.viewport_height());
    }

    [[nodiscard]] std::optional<screen_point> world_to_screen(
        const point2<REAL>& point) const noexcept {
        const REAL canvas_scale = scale();
        const REAL x = static_cast<REAL>(view_.image_width()) / REAL(2) +
            (point.x() - view_.center().x()) * canvas_scale;
        const REAL y = static_cast<REAL>(view_.image_height()) / REAL(2) -
            (point.y() - view_.center().y()) * canvas_scale;
        if (!std::isfinite(x) || !std::isfinite(y)) {
            return std::nullopt;
        }
        return screen_point{x, y};
    }

    void append_screen_line(
        const char* class_name,
        const screen_point& first,
        const screen_point& second,
        const char* stroke,
        REAL opacity,
        REAL stroke_width = REAL(0)) {
        elements_ << "<line class=\"" << class_name << "\" x1=\""
                  << first.x << "\" y1=\"" << first.y << "\" x2=\""
                  << second.x << "\" y2=\"" << second.y
                  << "\" stroke=\"" << stroke << "\" stroke-opacity=\""
                  << opacity << "\"";
        if (stroke_width > REAL(0)) {
            elements_ << " stroke-width=\"" << stroke_width << "\"";
        }
        elements_ << "/>\n";
    }

    void append_world_segment(
        const char* class_name,
        const point2<REAL>& first,
        const point2<REAL>& second,
        const char* stroke,
        REAL opacity,
        REAL stroke_width = REAL(0)) {
        const auto screen_first = world_to_screen(first);
        const auto screen_second = world_to_screen(second);
        if (!screen_first || !screen_second) {
            return;
        }
        append_screen_line(
            class_name,
            *screen_first,
            *screen_second,
            stroke,
            opacity,
            stroke_width);
    }

    [[nodiscard]] std::optional<std::pair<screen_point, screen_point>>
    projected_infinite_line(const ray2<REAL>& ray) const noexcept {
        const auto first = world_to_screen(ray.origin());
        const auto second = world_to_screen(ray.origin() + ray.direction());
        if (!first || !second) {
            return std::nullopt;
        }
        return clip_infinite_screen_line(*first, *second);
    }

    [[nodiscard]] std::optional<std::pair<screen_point, screen_point>>
    clip_infinite_screen_line(
        const screen_point& first,
        const screen_point& second) const noexcept {
        const REAL width = static_cast<REAL>(view_.image_width());
        const REAL height = static_cast<REAL>(view_.image_height());
        const REAL delta_x = second.x - first.x;
        const REAL delta_y = second.y - first.y;
        const REAL epsilon = REAL(128) * std::numeric_limits<REAL>::epsilon() *
            std::max({width, height, REAL(1)});

        std::array<screen_point, 4> contacts{};
        std::size_t contact_count = 0;
        const auto add_contact = [&](screen_point candidate) {
            candidate.x = std::clamp(candidate.x, REAL(0), width);
            candidate.y = std::clamp(candidate.y, REAL(0), height);
            for (std::size_t index = 0; index < contact_count; ++index) {
                const REAL dx = contacts[index].x - candidate.x;
                const REAL dy = contacts[index].y - candidate.y;
                if (dx * dx + dy * dy <= epsilon * epsilon) {
                    return;
                }
            }
            if (contact_count < contacts.size()) {
                contacts[contact_count++] = candidate;
            }
        };

        if (std::abs(delta_x) > epsilon) {
            for (REAL edge_x : {REAL(0), width}) {
                const REAL parameter = (edge_x - first.x) / delta_x;
                const REAL edge_y = first.y + parameter * delta_y;
                if (edge_y >= -epsilon && edge_y <= height + epsilon) {
                    add_contact({edge_x, edge_y});
                }
            }
        }
        if (std::abs(delta_y) > epsilon) {
            for (REAL edge_y : {REAL(0), height}) {
                const REAL parameter = (edge_y - first.y) / delta_y;
                const REAL edge_x = first.x + parameter * delta_x;
                if (edge_x >= -epsilon && edge_x <= width + epsilon) {
                    add_contact({edge_x, edge_y});
                }
            }
        }
        if (contact_count < 2) {
            return std::nullopt;
        }

        REAL greatest_distance = REAL(-1);
        std::pair<screen_point, screen_point> result{contacts[0], contacts[1]};
        for (std::size_t first_index = 0;
             first_index < contact_count;
             ++first_index) {
            for (std::size_t second_index = first_index + 1;
                 second_index < contact_count;
                 ++second_index) {
                const REAL dx = contacts[first_index].x - contacts[second_index].x;
                const REAL dy = contacts[first_index].y - contacts[second_index].y;
                const REAL separation = dx * dx + dy * dy;
                if (separation > greatest_distance) {
                    greatest_distance = separation;
                    result = {contacts[first_index], contacts[second_index]};
                }
            }
        }
        return result;
    }

    void append_location_dot(
        const char* class_name,
        const point2<REAL>& point,
        const char* fill,
        REAL radius_scale = REAL(1.75)) {
        const auto screen = world_to_screen(point);
        if (!screen) {
            return;
        }
        elements_ << "<circle class=\"" << class_name << "\" cx=\""
                  << screen->x << "\" cy=\"" << screen->y
                  << "\" r=\"" << options_.line_width * radius_scale
                  << "\" fill=\"" << fill
                  << "\" stroke=\"#202020\" stroke-width=\""
                  << options_.line_width * REAL(0.5) << "\"/>\n";
    }

    void append_axis(
        const char* class_name,
        const point2<REAL>& endpoint,
        const char* color,
        char label) {
        append_world_segment(
            class_name,
            point2<REAL>::origin(),
            endpoint,
            color,
            REAL(1),
            options_.line_width * REAL(0.5));
        const auto screen_endpoint = world_to_screen(endpoint);
        if (!screen_endpoint) {
            return;
        }
        const REAL offset = options_.line_width * REAL(2.5);
        elements_ << "<text class=\"" << class_name << "_label\" x=\""
                  << screen_endpoint->x + offset << "\" y=\""
                  << screen_endpoint->y - offset << "\" fill=\"" << color
                  << "\" stroke=\"none\" font-family=\"sans-serif\" "
                  << "font-size=\"" << options_.line_width * REAL(7)
                  << "\" font-weight=\"600\" text-anchor=\"middle\" "
                  << "dominant-baseline=\"middle\">" << label << "</text>\n";
    }

    void draw_coordinate_system() {
        append_axis(
            "nurbspath_canvas_axis_x",
            point2<REAL>::origin() + vector2<REAL>::unit_x(),
            "#e53935",
            'X');
        append_axis(
            "nurbspath_canvas_axis_y",
            point2<REAL>::origin() + vector2<REAL>::unit_y(),
            "#43a047",
            'Y');
    }

    svg_view2<REAL> view_;
    svg_graphics_options2<REAL> options_;
    std::ostringstream elements_;
};

/**
 * @brief Render one native 2D entity to a standalone flat SVG document.
 * @tparam REAL Floating-point scalar type.
 * @tparam ENTITY One of point2, ray2, circle2, or nurbs_spline2.
 * @param view Flat 2D viewport definition.
 * @param entity Entity in the independent 2D world.
 * @param options Solid-line styling and spline tessellation controls.
 * @return Complete self-contained flat SVG XML string.
 * @throws std::domain_error When entity is an unlimited repeated spline.
 */
template <std::floating_point REAL, typename ENTITY>
    requires detail::svg_canvas_entity2<ENTITY, REAL>
[[nodiscard]] std::string to_svg(
    const svg_view2<REAL>& view,
    const ENTITY& entity,
    const svg_graphics_options2<REAL>& options = {}) {
    svg_canvas2<REAL> canvas(view, options);
    canvas.add(entity);
    return canvas.svg();
}

/**
 * @brief Accumulate projected geometry and emit a self-contained SVG document.
 *
 * Entities render in `add()` order. No `add(vector3)` overload exists because a
 * free vector has no world-space position.
 *
 * @tparam REAL Floating-point scalar type.
 */
template <std::floating_point REAL>
class svg_document3 {
public:
    /**
     * @brief Construct an empty SVG scene containing the world XYZ reference.
     * @param view Validated projection and viewport definition.
     * @param options Styling and tessellation controls.
     * @throws std::invalid_argument When options are invalid.
     */
    explicit svg_document3(
        const svg_view3<REAL>& view,
        const svg_graphics_options<REAL>& options = {})
        : view_(view), options_(options), frame_(make_camera_frame(view)) {
        options_.validate();
        elements_.imbue(std::locale::classic());
        elements_ << std::setprecision(std::numeric_limits<REAL>::max_digits10);
        // Every diagnostic scene begins with the same world-space reference.
        // Drawing it first lets subsequently added geometry remain prominent.
        draw_world_coordinate_system();
    }

    /**
     * @brief Add a point as a black dot of diameter twice the line width.
     * @param point World-space point.
     */
    void add(const point3<REAL>& point) {
        const auto projected = project_world_point(point);
        if (!projected) {
            return;
        }
        elements_ << "<circle class=\"nurbspath_point\" cx=\""
                  << projected->x << "\" cy=\"" << projected->y
                  << "\" r=\"" << options_.line_width
                  << "\" fill=\"#000000\" stroke=\"none\"/>\n";
    }

    /**
     * @brief Add a dashed supporting line, positive unit direction, and origin.
     * @param ray World-space ray to visualize.
     */
    void add(const ray3<REAL>& ray) {
        const auto screen_line = projected_infinite_line(ray);
        if (screen_line) {
            append_screen_line(
                "nurbspath_ray",
                screen_line->first,
                screen_line->second,
                "#e7a0a0",
                REAL(0.9),
                line_style::dashed);
        }
        append_world_segment(
            "nurbspath_ray_positive_direction",
            ray.origin(),
            ray.origin() + ray.direction().normalized(REAL(0)),
            "#d65c5c",
            REAL(1));
        append_location_dot(
            "nurbspath_ray_origin", ray.origin(), "#d65c5c");
    }

    /**
     * @brief Add a sphere silhouette, visible/hidden equator, and center dot.
     * @param sphere World-space sphere to visualize.
     */
    void add(const sphere3<REAL>& sphere) {
        draw_sphere_outline(sphere);
        draw_sphere_equator(sphere);
        append_location_dot(
            "nurbspath_sphere_center", sphere.center(), "#4f8fcf");
    }

    /**
     * @brief Add a plane square, positive parameter axes, normal, and origin.
     * @param plane Infinite plane to visualize at its parameter origin.
     */
    void add(const plane3<REAL>& plane) {
        const REAL half_size = options_.plane_square_side_length / REAL(2);
        const point3<REAL> first =
            plane.origin() - half_size * plane.u_direction() -
            half_size * plane.v_direction();
        const point3<REAL> second =
            plane.origin() + half_size * plane.u_direction() -
            half_size * plane.v_direction();
        const point3<REAL> third =
            plane.origin() + half_size * plane.u_direction() +
            half_size * plane.v_direction();
        const point3<REAL> fourth =
            plane.origin() - half_size * plane.u_direction() +
            half_size * plane.v_direction();

        append_world_segment("nurbspath_plane_square", first, second, "#9bd3a6", REAL(1));
        append_world_segment("nurbspath_plane_square", second, third, "#9bd3a6", REAL(1));
        append_world_segment("nurbspath_plane_square", third, fourth, "#9bd3a6", REAL(1));
        append_world_segment("nurbspath_plane_square", fourth, first, "#9bd3a6", REAL(1));
        append_world_segment(
            "nurbspath_plane_normal",
            plane.origin(),
            plane.origin() + plane.normal(),
            "#78bd89",
            REAL(1));
        append_world_segment(
            "nurbspath_plane_u_direction",
            plane.origin(),
            plane.origin() +
                options_.plane_parameter_axis_length * plane.u_direction(),
            "#4eaa6c",
            REAL(1));
        append_world_segment(
            "nurbspath_plane_v_direction",
            plane.origin(),
            plane.origin() +
                options_.plane_parameter_axis_length * plane.v_direction(),
            "#6fbd83",
            REAL(1));
        append_location_dot(
            "nurbspath_plane_origin", plane.origin(), "#54a56a");
    }

    /**
     * @brief Add control geometry, a tessellated curve, and endpoint dots.
     * @param spline NURBS curve to visualize over its active domain.
     * @throws std::domain_error When the spline repeats without a finite limit.
     */
    void add(const nurbs_spline3<REAL>& spline) {
        if (spline.period_count() == 0) {
            throw std::domain_error(
                "SVG rendering requires a finite spline domain");
        }
        const auto& control_points = spline.control_points();
        for (std::size_t index = 1; index < control_points.size(); ++index) {
            append_world_dashed_segment(
                "nurbspath_spline_control_polygon",
                control_points[index - 1],
                control_points[index],
                "#b8a6cc",
                REAL(0.6));
        }
        for (const point3<REAL>& control_point : control_points) {
            append_location_dot(
                "nurbspath_spline_control_point",
                control_point,
                "#80649b",
                REAL(1.4));
        }

        point3<REAL> previous = spline.evaluate(spline.s_min());
        for (std::size_t index = 1;
             index <= options_.spline_segment_count;
             ++index) {
            const REAL fraction = static_cast<REAL>(index) /
                                  static_cast<REAL>(options_.spline_segment_count);
            const REAL s = spline.s_min() + fraction * (spline.s_max() - spline.s_min());
            const point3<REAL> current = spline.evaluate(s);
            append_world_segment(
                "nurbspath_spline", previous, current, "#000000", REAL(1));
            previous = current;
        }

        // Endpoint dots are drawn last and slightly larger. This keeps them
        // identifiable when a clamped spline endpoint equals a control point.
        append_location_dot(
            "nurbspath_spline_endpoint",
            spline.evaluate(spline.s_min()),
            "#000000",
            REAL(1.9));
        append_location_dot(
            "nurbspath_spline_endpoint",
            spline.evaluate(spline.s_max()),
            "#000000",
            REAL(1.9));
    }

    /**
     * @brief Project and add a 2D point as a black point marker.
     * @param plane Plane whose parameter frame embeds the independent 2D world.
     * @param point Point in the 2D world.
     */
    void add(const plane3<REAL>& plane, const point2<REAL>& point) {
        const auto projected = project_world_point(project(plane, point));
        if (!projected) {
            return;
        }
        elements_ << "<circle class=\"nurbspath_point2\" cx=\""
                  << projected->x << "\" cy=\"" << projected->y
                  << "\" r=\"" << options_.line_width
                  << "\" fill=\"#000000\" stroke=\"none\"/>\n";
    }

    /**
     * @brief Project and add a 2D ray with pale-red patterned strokes.
     *
     * The infinite supporting line is dash-dotted and the positive unit
     * direction marker is dotted.
     *
     * @param plane Plane whose parameter frame embeds the independent 2D world.
     * @param ray Ray in the 2D world.
     */
    void add(const plane3<REAL>& plane, const ray2<REAL>& ray) {
        const ray3<REAL> projected_ray = project(plane, ray);
        const auto screen_line = projected_infinite_line(projected_ray);
        if (screen_line) {
            append_screen_line(
                "nurbspath_ray2",
                screen_line->first,
                screen_line->second,
                "#e7a0a0",
                REAL(0.9),
                line_style::dash_dotted);
        }
        append_world_segment(
            "nurbspath_ray2_positive_direction",
            projected_ray.origin(),
            projected_ray.origin() +
                projected_ray.direction().normalized(REAL(0)),
            "#d65c5c",
            REAL(1),
            line_style::dotted);
        append_location_dot(
            "nurbspath_ray2_origin", projected_ray.origin(), "#d65c5c");
    }

    /**
     * @brief Project and add a 2D circle as a patterned center/radius sphere.
     *
     * The outline and visible equator are dash-dotted; the hidden equator is
     * dotted. Colors remain identical to `sphere3`.
     *
     * @param plane Plane used to project the independent 2D circle center.
     * @param circle Circle in the 2D world.
     */
    void add(const plane3<REAL>& plane, const circle2<REAL>& circle) {
        const sphere3<REAL> sphere = project(plane, circle);
        draw_sphere_outline(
            sphere, "nurbspath_circle2_outline", line_style::dash_dotted);
        draw_sphere_equator(
            sphere,
            "nurbspath_circle2_equator_visible",
            "nurbspath_circle2_equator_hidden",
            line_style::dash_dotted,
            line_style::dotted);
        append_location_dot(
            "nurbspath_circle2_center", sphere.center(), "#4f8fcf");
    }

    /**
     * @brief Project and add a 2D spline using patterned black/violet strokes.
     *
     * The evaluated curve is dash-dotted and its control polygon is dotted.
     * Control and endpoint dots are unchanged.
     *
     * @param plane Plane whose parameter frame embeds the independent 2D world.
     * @param spline NURBS spline in the 2D world.
     * @throws std::domain_error When the spline repeats without a finite limit.
     */
    void add(const plane3<REAL>& plane, const nurbs_spline2<REAL>& spline) {
        if (spline.period_count() == 0) {
            throw std::domain_error(
                "SVG rendering requires a finite 2D spline domain");
        }
        const nurbs_spline3<REAL> projected_spline = project(plane, spline);
        const auto& control_points = projected_spline.control_points();
        for (std::size_t index = 1; index < control_points.size(); ++index) {
            append_world_segment(
                "nurbspath_spline2_control_polygon",
                control_points[index - 1],
                control_points[index],
                "#b8a6cc",
                REAL(0.6),
                line_style::dotted);
        }
        for (const point3<REAL>& control_point : control_points) {
            append_location_dot(
                "nurbspath_spline2_control_point",
                control_point,
                "#80649b",
                REAL(1.4));
        }

        point3<REAL> previous = projected_spline.evaluate(projected_spline.s_min());
        for (std::size_t index = 1;
             index <= options_.spline_segment_count;
             ++index) {
            const REAL fraction = static_cast<REAL>(index) /
                                  static_cast<REAL>(options_.spline_segment_count);
            const REAL s = projected_spline.s_min() +
                fraction * (projected_spline.s_max() - projected_spline.s_min());
            const point3<REAL> current = projected_spline.evaluate(s);
            append_world_segment(
                "nurbspath_spline2",
                previous,
                current,
                "#000000",
                REAL(1),
                line_style::dash_dotted);
            previous = current;
        }

        append_location_dot(
            "nurbspath_spline2_endpoint",
            projected_spline.evaluate(projected_spline.s_min()),
            "#000000",
            REAL(1.9));
        append_location_dot(
            "nurbspath_spline2_endpoint",
            projected_spline.evaluate(projected_spline.s_max()),
            "#000000",
            REAL(1.9));
    }

    /** @brief Serialize the complete scene. @return Self-contained SVG XML string. */
    [[nodiscard]] std::string svg() const {
        std::ostringstream output;
        output.imbue(std::locale::classic());
        output << std::setprecision(std::numeric_limits<REAL>::max_digits10);
        output << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
               << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\""
               << view_.image_width() << "\" height=\"" << view_.image_height()
               << "\" viewBox=\"0 0 " << view_.image_width() << ' '
               << view_.image_height() << "\">\n"
               << "<defs><clipPath id=\"nurbspath_viewport\"><rect x=\"0\" y=\"0\" width=\""
               << view_.image_width() << "\" height=\"" << view_.image_height()
               << "\"/></clipPath></defs>\n"
               << "<g clip-path=\"url(#nurbspath_viewport)\" fill=\"none\" "
               << "stroke-width=\"" << options_.line_width
               << "\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n"
               << elements_.str()
               << "</g>\n</svg>\n";
        return output.str();
    }

    /**
     * @brief Stream the complete SVG document.
     * @param output Destination stream.
     */
    void write(std::ostream& output) const {
        output << svg();
    }

private:
    using camera_frame = detail::svg_camera_frame3<REAL>;
    using camera_point = detail::svg_camera_point3<REAL>;
    using screen_point = detail::svg_screen_point3<REAL>;
    using line_style = detail::svg_line_style;

    [[nodiscard]] static camera_frame make_camera_frame(
        const svg_view3<REAL>& view) {
        const vector3<REAL> look = view.lookatpoint() - view.eyepoint();
        const REAL look_distance = look.length();
        const vector3<REAL> forward = look / look_distance;
        const vector3<REAL> preferred_up =
            std::abs(forward.dot(vector3<REAL>::unit_z())) < REAL(0.95)
                ? vector3<REAL>::unit_z()
                : vector3<REAL>::unit_y();
        const vector3<REAL> right = forward.cross(preferred_up).normalized();
        const vector3<REAL> up = right.cross(forward).normalized();
        return {forward, right, up, look_distance};
    }

    [[nodiscard]] camera_point world_to_camera(const point3<REAL>& point) const noexcept {
        const vector3<REAL> offset = point - view_.eyepoint();
        return {
            offset.dot(frame_.right),
            offset.dot(frame_.up),
            offset.dot(frame_.forward)
        };
    }

    [[nodiscard]] camera_point world_direction_to_camera(
        const vector3<REAL>& direction) const noexcept {
        return {
            direction.dot(frame_.right),
            direction.dot(frame_.up),
            direction.dot(frame_.forward)
        };
    }

    [[nodiscard]] std::optional<screen_point> project_camera_point(
        const camera_point& point) const noexcept {
        REAL screen_x = REAL(0);
        REAL screen_y = REAL(0);
        const REAL image_width = static_cast<REAL>(view_.image_width());
        const REAL image_height = static_cast<REAL>(view_.image_height());

        if (view_.projection() == svg_projection::orthographic) {
            // Use one scale for both axes so circles remain circles even when
            // viewport and image aspect ratios differ.  Both requested world
            // extents remain visible; the looser axis is simply letterboxed.
            const REAL scale = std::min(
                image_width / view_.viewport_width(),
                image_height / view_.viewport_height());
            screen_x = image_width / REAL(2) + point.x * scale;
            screen_y = image_height / REAL(2) - point.y * scale;
        } else {
            if (point.z < view_.near_distance()) {
                return std::nullopt;
            }
            const REAL half_height =
                point.z * std::tan(view_.vertical_view_angle() / REAL(2));
            const REAL aspect_ratio = image_width / image_height;
            const REAL half_width = half_height * aspect_ratio;
            screen_x = image_width / REAL(2) * (REAL(1) + point.x / half_width);
            screen_y = image_height / REAL(2) * (REAL(1) - point.y / half_height);
        }

        if (!std::isfinite(screen_x) || !std::isfinite(screen_y)) {
            return std::nullopt;
        }
        return screen_point{screen_x, screen_y};
    }

    [[nodiscard]] std::optional<screen_point> project_world_point(
        const point3<REAL>& point) const noexcept {
        return project_camera_point(world_to_camera(point));
    }

    /// Clip a world segment against the perspective near plane before project.
    [[nodiscard]] std::optional<std::pair<screen_point, screen_point>>
    project_world_segment(
        const point3<REAL>& first,
        const point3<REAL>& second) const noexcept {
        camera_point camera_first = world_to_camera(first);
        camera_point camera_second = world_to_camera(second);

        if (view_.projection() == svg_projection::perspective) {
            const REAL near_distance = view_.near_distance();
            if (camera_first.z < near_distance && camera_second.z < near_distance) {
                return std::nullopt;
            }

            const auto clip_endpoint = [near_distance](
                                           camera_point& hidden,
                                           const camera_point& visible) {
                const REAL fraction = (near_distance - hidden.z) /
                                      (visible.z - hidden.z);
                hidden.x += fraction * (visible.x - hidden.x);
                hidden.y += fraction * (visible.y - hidden.y);
                hidden.z = near_distance;
            };
            if (camera_first.z < near_distance) {
                clip_endpoint(camera_first, camera_second);
            } else if (camera_second.z < near_distance) {
                clip_endpoint(camera_second, camera_first);
            }
        }

        const auto projected_first = project_camera_point(camera_first);
        const auto projected_second = project_camera_point(camera_second);
        if (!projected_first || !projected_second) {
            return std::nullopt;
        }
        return std::pair{*projected_first, *projected_second};
    }

    [[nodiscard]] std::optional<std::pair<screen_point, screen_point>>
    projected_infinite_line(const ray3<REAL>& ray) const noexcept {
        camera_point first{};
        camera_point second{};
        const camera_point origin = world_to_camera(ray.origin());
        const camera_point direction = world_direction_to_camera(ray.direction());
        const REAL direction_scale = std::max({
            std::abs(direction.x), std::abs(direction.y), std::abs(direction.z)});
        const REAL numerical_zero =
            REAL(64) * std::numeric_limits<REAL>::epsilon() *
            std::max(direction_scale, REAL(1));

        if (view_.projection() == svg_projection::orthographic) {
            first = origin;
            second = {
                origin.x + direction.x,
                origin.y + direction.y,
                origin.z + direction.z
            };
        } else if (std::abs(direction.z) > numerical_zero) {
            // Select two positive-depth points on the infinite supporting line.
            // Their projected line is then extended exactly to the SVG bounds.
            const REAL first_depth = view_.near_distance() * REAL(2);
            const REAL second_depth = first_depth +
                std::max(frame_.look_distance, view_.near_distance() * REAL(2));
            const REAL first_s = (first_depth - origin.z) / direction.z;
            const REAL second_s = (second_depth - origin.z) / direction.z;
            first = {
                origin.x + first_s * direction.x,
                origin.y + first_s * direction.y,
                first_depth
            };
            second = {
                origin.x + second_s * direction.x,
                origin.y + second_s * direction.y,
                second_depth
            };
        } else {
            if (origin.z < view_.near_distance()) {
                return std::nullopt;
            }
            first = origin;
            second = {
                origin.x + direction.x,
                origin.y + direction.y,
                origin.z + direction.z
            };
        }

        const auto projected_first = project_camera_point(first);
        const auto projected_second = project_camera_point(second);
        if (!projected_first || !projected_second) {
            return std::nullopt;
        }
        return clip_infinite_screen_line(*projected_first, *projected_second);
    }

    /// Intersect an infinite 2D line with all four SVG viewport edges.
    [[nodiscard]] std::optional<std::pair<screen_point, screen_point>>
    clip_infinite_screen_line(
        const screen_point& first,
        const screen_point& second) const noexcept {
        const REAL width = static_cast<REAL>(view_.image_width());
        const REAL height = static_cast<REAL>(view_.image_height());
        const REAL delta_x = second.x - first.x;
        const REAL delta_y = second.y - first.y;
        const REAL epsilon = REAL(128) * std::numeric_limits<REAL>::epsilon() *
            std::max({width, height, REAL(1)});

        std::array<screen_point, 4> contacts{};
        std::size_t contact_count = 0;
        const auto add_contact = [&](screen_point candidate) {
            candidate.x = std::clamp(candidate.x, REAL(0), width);
            candidate.y = std::clamp(candidate.y, REAL(0), height);
            for (std::size_t index = 0; index < contact_count; ++index) {
                const REAL x_difference = contacts[index].x - candidate.x;
                const REAL y_difference = contacts[index].y - candidate.y;
                if (x_difference * x_difference + y_difference * y_difference <=
                    epsilon * epsilon) {
                    return;
                }
            }
            if (contact_count < contacts.size()) {
                contacts[contact_count++] = candidate;
            }
        };

        if (std::abs(delta_x) > epsilon) {
            for (REAL edge_x : {REAL(0), width}) {
                const REAL parameter = (edge_x - first.x) / delta_x;
                const REAL edge_y = first.y + parameter * delta_y;
                if (edge_y >= -epsilon && edge_y <= height + epsilon) {
                    add_contact({edge_x, edge_y});
                }
            }
        }
        if (std::abs(delta_y) > epsilon) {
            for (REAL edge_y : {REAL(0), height}) {
                const REAL parameter = (edge_y - first.y) / delta_y;
                const REAL edge_x = first.x + parameter * delta_x;
                if (edge_x >= -epsilon && edge_x <= width + epsilon) {
                    add_contact({edge_x, edge_y});
                }
            }
        }
        if (contact_count < 2) {
            return std::nullopt;
        }

        // A corner can provide more than two numerically distinct candidates.
        // Select the farthest pair to span the complete viewport line.
        REAL greatest_distance = REAL(-1);
        std::pair<screen_point, screen_point> result{contacts[0], contacts[1]};
        for (std::size_t first_index = 0;
             first_index < contact_count;
             ++first_index) {
            for (std::size_t second_index = first_index + 1;
                 second_index < contact_count;
                 ++second_index) {
                const REAL x_difference =
                    contacts[first_index].x - contacts[second_index].x;
                const REAL y_difference =
                    contacts[first_index].y - contacts[second_index].y;
                const REAL distance_squared =
                    x_difference * x_difference + y_difference * y_difference;
                if (distance_squared > greatest_distance) {
                    greatest_distance = distance_squared;
                    result = {contacts[first_index], contacts[second_index]};
                }
            }
        }
        return result;
    }

    void append_screen_line(
        const char* class_name,
        const screen_point& first,
        const screen_point& second,
        const char* stroke,
        REAL opacity,
        line_style style = line_style::solid,
        REAL stroke_width = REAL(0)) {
        elements_ << "<line class=\"" << class_name << "\" x1=\""
                  << first.x << "\" y1=\"" << first.y << "\" x2=\""
                  << second.x << "\" y2=\"" << second.y
                  << "\" stroke=\"" << stroke << "\" stroke-opacity=\""
                  << opacity << "\"";
        switch (style) {
        case line_style::solid:
            break;
        case line_style::dashed:
            elements_ << " stroke-dasharray=\""
                      << options_.line_width * REAL(4) << ' '
                      << options_.line_width * REAL(3) << "\"";
            break;
        case line_style::dotted:
            elements_ << " stroke-dasharray=\""
                      << options_.line_width * REAL(0.5) << ' '
                      << options_.line_width * REAL(2.5) << "\"";
            break;
        case line_style::dash_dotted:
            elements_ << " stroke-dasharray=\""
                      << options_.line_width * REAL(5) << ' '
                      << options_.line_width * REAL(2.5) << ' '
                      << options_.line_width * REAL(0.5) << ' '
                      << options_.line_width * REAL(2.5) << "\"";
            break;
        }
        if (stroke_width > REAL(0)) {
            elements_ << " stroke-width=\"" << stroke_width << "\"";
        }
        elements_ << "/>\n";
    }

    /// Draw one positive world axis and place its letter just beyond the tip.
    void append_world_axis(
        const char* class_name,
        const point3<REAL>& endpoint,
        const char* color,
        char label) {
        const point3<REAL> world_origin = point3<REAL>::origin();
        const auto projected = project_world_segment(world_origin, endpoint);
        if (projected) {
            append_screen_line(
                class_name,
                projected->first,
                projected->second,
                color,
                REAL(1),
                line_style::solid,
                options_.line_width * REAL(0.5));
        }

        const auto projected_endpoint = project_world_point(endpoint);
        if (!projected_endpoint) {
            return;
        }
        const REAL label_offset = options_.line_width * REAL(2.5);
        const REAL font_size = options_.line_width * REAL(7);
        elements_ << "<text class=\"" << class_name << "_label\" x=\""
                  << projected_endpoint->x + label_offset << "\" y=\""
                  << projected_endpoint->y - label_offset
                  << "\" fill=\"" << color
                  << "\" stroke=\"none\" font-family=\"sans-serif\" "
                  << "font-size=\"" << font_size
                  << "\" font-weight=\"600\" text-anchor=\"middle\" "
                  << "dominant-baseline=\"middle\">" << label << "</text>\n";
    }

    /// Add a unit positive XYZ coordinate system at the Cartesian world origin.
    void draw_world_coordinate_system() {
        append_world_axis(
            "nurbspath_world_axis_x",
            point3<REAL>::origin() + vector3<REAL>::unit_x(),
            "#e53935",
            'X');
        append_world_axis(
            "nurbspath_world_axis_y",
            point3<REAL>::origin() + vector3<REAL>::unit_y(),
            "#43a047",
            'Y');
        append_world_axis(
            "nurbspath_world_axis_z",
            point3<REAL>::origin() + vector3<REAL>::unit_z(),
            "#1e88e5",
            'Z');
    }

    void append_world_segment(
        const char* class_name,
        const point3<REAL>& first,
        const point3<REAL>& second,
        const char* stroke,
        REAL opacity,
        line_style style = line_style::solid) {
        const auto projected = project_world_segment(first, second);
        if (!projected) {
            return;
        }
        append_screen_line(
            class_name,
            projected->first,
            projected->second,
            stroke,
            opacity,
            style);
    }

    void append_world_dashed_segment(
        const char* class_name,
        const point3<REAL>& first,
        const point3<REAL>& second,
        const char* stroke,
        REAL opacity) {
        append_world_segment(
            class_name, first, second, stroke, opacity, line_style::dashed);
    }

    /// Entity location dots are intentionally larger and more saturated than
    /// their pale construction lines. A narrow dark rim keeps the location
    /// visible on transparent SVGs displayed over either light or dark pages.
    void append_location_dot(
        const char* class_name,
        const point3<REAL>& point,
        const char* fill,
        REAL radius_scale = REAL(1.75)) {
        const auto projected = project_world_point(point);
        if (!projected) {
            return;
        }
        elements_ << "<circle class=\"" << class_name << "\" cx=\""
                  << projected->x << "\" cy=\"" << projected->y
                  << "\" r=\"" << options_.line_width * radius_scale
                  << "\" fill=\"" << fill
                  << "\" stroke=\"#202020\" stroke-width=\""
                  << options_.line_width * REAL(0.5) << "\"/>\n";
    }

    [[nodiscard]] vector3<REAL> perpendicular_axis(
        const vector3<REAL>& normal) const {
        vector3<REAL> candidate = frame_.right.rejected_from(normal, REAL(0));
        if (candidate.is_near_zero()) {
            candidate = frame_.up.rejected_from(normal, REAL(0));
        }
        return candidate.normalized();
    }

    void draw_sphere_outline(
        const sphere3<REAL>& sphere,
        const char* class_name = "nurbspath_sphere_outline",
        line_style style = line_style::solid) {
        point3<REAL> circle_center = sphere.center();
        REAL circle_radius = sphere.radius();
        vector3<REAL> circle_normal = -frame_.forward;

        if (view_.projection() == svg_projection::perspective) {
            const vector3<REAL> center_to_eye = view_.eyepoint() - sphere.center();
            const REAL eye_distance = center_to_eye.length();
            if (eye_distance > sphere.radius()) {
                circle_normal = center_to_eye / eye_distance;
                const REAL radius_ratio = sphere.radius() / eye_distance;
                circle_center = sphere.center() +
                    (radius_ratio * radius_ratio) * center_to_eye;
                circle_radius = sphere.radius() *
                    std::sqrt(std::max(REAL(0), REAL(1) - radius_ratio * radius_ratio));
            } else if (eye_distance > REAL(0)) {
                // Inside/on-sphere cameras have no mathematical silhouette.
                // A camera-facing great circle remains a useful debug marker.
                circle_normal = center_to_eye / eye_distance;
            }
        }

        const vector3<REAL> first_axis = perpendicular_axis(circle_normal);
        const vector3<REAL> second_axis =
            circle_normal.cross(first_axis).normalized();
        point3<REAL> previous = circle_center + circle_radius * first_axis;
        for (std::size_t index = 1;
             index <= options_.sphere_segment_count;
             ++index) {
            const REAL angle = REAL(2) * std::numbers::pi_v<REAL> *
                static_cast<REAL>(index) /
                static_cast<REAL>(options_.sphere_segment_count);
            const point3<REAL> current = circle_center + circle_radius *
                (std::cos(angle) * first_axis + std::sin(angle) * second_axis);
            append_world_segment(
                class_name, previous, current, "#8fc5e8", REAL(1), style);
            previous = current;
        }
    }

    void draw_sphere_equator(
        const sphere3<REAL>& sphere,
        const char* visible_class = "nurbspath_sphere_equator_visible",
        const char* hidden_class = "nurbspath_sphere_equator_hidden",
        line_style visible_style = line_style::solid,
        line_style hidden_style = line_style::solid) {
        point3<REAL> previous = sphere.point_at(REAL(0), REAL(0));
        for (std::size_t index = 1;
             index <= options_.sphere_segment_count;
             ++index) {
            const REAL previous_angle = REAL(2) * std::numbers::pi_v<REAL> *
                static_cast<REAL>(index - 1) /
                static_cast<REAL>(options_.sphere_segment_count);
            const REAL angle = REAL(2) * std::numbers::pi_v<REAL> *
                static_cast<REAL>(index) /
                static_cast<REAL>(options_.sphere_segment_count);
            const REAL middle_angle = (previous_angle + angle) / REAL(2);
            const point3<REAL> current = sphere.point_at(angle, REAL(0));
            const vector3<REAL> middle_normal{
                std::cos(middle_angle), std::sin(middle_angle), REAL(0)};
            const point3<REAL> middle_point =
                sphere.center() + sphere.radius() * middle_normal;
            const vector3<REAL> toward_camera =
                view_.projection() == svg_projection::orthographic
                    ? -frame_.forward
                    : view_.eyepoint() - middle_point;
            const bool visible = middle_normal.dot(toward_camera) >= REAL(0);
            append_world_segment(
                visible ? visible_class : hidden_class,
                previous,
                current,
                visible ? "#6aaed6" : "#c7e3f4",
                visible ? REAL(1) : REAL(0.65),
                visible ? visible_style : hidden_style);
            previous = current;
        }
    }

    svg_view3<REAL> view_;
    svg_graphics_options<REAL> options_;
    camera_frame frame_;
    std::ostringstream elements_;
};

/**
 * @brief Render one supported entity to a standalone SVG document.
 * @tparam REAL Floating-point scalar type.
 * @tparam ENTITY One of point3, ray3, sphere3, plane3, or nurbs_spline3.
 * @param view Projection and viewport definition.
 * @param entity Entity to render.
 * @param options Styling and tessellation controls.
 * @return Complete self-contained SVG XML string.
 * @throws std::domain_error When entity is an unlimited repeated spline.
 */
template <std::floating_point REAL, typename ENTITY>
[[nodiscard]] std::string to_svg(
    const svg_view3<REAL>& view,
    const ENTITY& entity,
    const svg_graphics_options<REAL>& options = {}) {
    svg_document3<REAL> document(view, options);
    document.add(entity);
    return document.svg();
}

/**
 * @brief Project one supported 2D entity through a plane and render it to SVG.
 * @tparam REAL Floating-point scalar type.
 * @tparam ENTITY One of point2, ray2, circle2, or nurbs_spline2.
 * @param view Projection and viewport definition for the 3D SVG scene.
 * @param plane Plane whose parameter frame embeds the 2D entity.
 * @param entity Entity in the independent 2D world.
 * @param options Styling and tessellation controls.
 * @return Complete self-contained SVG XML string.
 * @throws std::domain_error When entity is an unlimited repeated spline.
 */
template <std::floating_point REAL, typename ENTITY>
[[nodiscard]] std::string to_svg(
    const svg_view3<REAL>& view,
    const plane3<REAL>& plane,
    const ENTITY& entity,
    const svg_graphics_options<REAL>& options = {}) {
    svg_document3<REAL> document(view, options);
    document.add(plane, entity);
    return document.svg();
}

} // namespace nurbspath
