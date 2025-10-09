#pragma once
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include <vector>
#include <ranges>
#include <concepts>
#include <cassert>

namespace rz {


/**
 * @brief Immutable sequence of 2D waypoints with unit-safe geometry.
 *
 * `DiscretePath` stores a read-only ordered list of `Point` (meters). It supports:
 * - Construction from an initializer list or any C++20 range of points,
 * - Concatenation with another path or a single point,
 * - Const iteration (forward and reverse),
 * - Random access and front/back access,
 * - Geometric queries such as signed curvature at an interior vertex.
 *
 * @invariant Paths are **non-empty**. All constructors assert `size() > 0`.
 * @note This type is immutable: iteration returns const iterators and no mutating
 *       accessors are exposed.
 */
class DiscretePath {
    public:
    using value_type = Point;
    using const_reference = const Point&;
    using container_type = std::vector<Point>;
    using size_type = std::size_t;
    using const_iterator = std::vector<Point>::const_iterator;
    using const_reverse_iterator = std::vector<Point>::const_reverse_iterator;

    /**
     * @brief Constructs a path from a brace-initialized list of points.
     *
     * @param waypoints Non-empty list of waypoints (meters).
     * @pre `waypoints.size() > 0` (debug-checked).
     */
    explicit DiscretePath(std::initializer_list<Point> waypoints);

    /**
     * @brief Constructs a path from any C++20 input range of points.
     *
     * Accepts containers and views whose element/reference type is convertible to `Point`,
     * e.g. `std::vector<Point>`, `std::list<Point>`, raw arrays, or range pipelines
     * like `points | std::views::filter(...)`.
     *
     * @tparam Range A `std::ranges::input_range` whose reference is convertible to `Point`.
     * @param range Non-empty range of waypoints (meters).
     * @pre `std::ranges::distance(range) > 0` (debug-checked after construction).
     * @note This constructor is disabled for `Range = DiscretePath` to avoid hijacking copy/move.
     */
    template <std::ranges::input_range Range>
        requires (
            !std::same_as<std::remove_cvref_t<Range>, DiscretePath> &&
            std::convertible_to<std::ranges::range_reference_t<Range>, Point>
        )
    explicit DiscretePath(Range&& range)
        : path(std::ranges::begin(range), std::ranges::end(range)) {
        assert(!path.empty() && "DiscretePath cannot be empty");
    }
    
    /**
     * @brief Concatenates two paths (this followed by @p rhs).
     *
     * @param rhs Path to append.
     * @return New path containing `*this` then `rhs`.
     */
    DiscretePath operator+(const DiscretePath& rhs) const;

    /**
     * @brief Appends a single waypoint to the end of the path.
     *
     * @param rhs Point to append (meters).
     * @return New path with @p rhs appended.
     */
    DiscretePath operator+(const Point& rhs) const;

    /// @brief Forward begin iterator.
    const_iterator begin() const noexcept;

    /// @brief Forward end iterator (one past the last).
    const_iterator end() const noexcept;

    /// @brief Reverse begin iterator (last element).
    const_reverse_iterator rbegin() const noexcept;

    /// @brief Reverse end iterator (one before the first).
    const_reverse_iterator rend() const noexcept;

    /**
     * @brief Random access without bounds checking.
     * @param index Zero-based index in `[0, size())`.
     * @return Reference to the indexed point.
     */
    const_reference operator[](size_type index) const noexcept;

    /// @brief First waypoint.
    const_reference front() const noexcept;

    /// @brief Last waypoint.
    const_reference back() const noexcept;

    /**
     * @brief Signed curvature at vertex @p index using the circumcircle of three consecutive points.
     *
     * For an interior index `i` (i.e. `1 ≤ i ≤ size()-2`), let
     * `R = circumradius(P[i-1], P[i], P[i+1])`. The unsigned curvature is `κ = 1/R`.
     * The sign follows the triangle orientation (left turn positive, right turn negative).
     * Endpoints and collinear triples yield 0.
     *
     * @param index Vertex index at which to evaluate curvature.
     * @return Curvature in 1/meters (`au::QuantityD<au::Inverse<au::Meters>>`).
     */
    au::QuantityD<au::Inverse<au::Meters>> getCurvature(std::size_t index) const noexcept;

    /// @brief Number of waypoints.
    size_type size() const noexcept;

    private:
    std::vector<Point> path;
};

/**
 * @brief Iterator-based closest-point search by Euclidean distance.
 *
 * Returns an iterator to the element in `[begin, end)` minimizing `distTo(point)`.
 * If the range is empty, returns `end`.
 *
 * @param begin Const iterator to the first waypoint.
 * @param end   Const iterator one past the last waypoint.
 * @param point Query point (meters).
 * @return Iterator to the closest waypoint, or @p end if the range is empty.
 */
DiscretePath::const_iterator closestPoint(DiscretePath::const_iterator begin,
                                          DiscretePath::const_iterator end, const Point& point);
} // namespace rz
