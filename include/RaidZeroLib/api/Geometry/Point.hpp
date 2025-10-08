#pragma once
#include "au/au.hpp"
#include <optional>

namespace rz {

class Rotation;

/**
 * @brief Immutable 2D point / vector with unit-safe Cartesian coordinates.
 *
 * `Point` stores an (x, y) position in meters and provides common vector
 * operations (addition, subtraction, scaling, dot / wedge products, projection), 
 * geometric queries (magnitude, direction, distance, angles), and rotation by a 
 * `Rotation`.
 *
 * All operations preserve units via `au` quantities. Overloads that take
 * `double` scalars are unitless multipliers/divisors applied to the coordinates.
 */
class Point {
    public:
    /**
     * @brief Constructs the origin (0 m, 0 m).
     */
    constexpr Point() noexcept = default;


    /**
     * @brief Constructs a point from Cartesian components.
     *
     * @param x X component (length units, e.g., meters).
     * @param y Y component (length units, e.g., meters).
     */
    Point(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y) noexcept;

    /**
     * @brief Constructs a point from polar form.
     *
     * Forms the vector with magnitude `magnitude` and direction `rotation`:
     * `x = magnitude * cos(rotation)`, `y = magnitude * sin(rotation)`.
     *
     * @param magnitude Vector length (meters).
     * @param rotation Heading as a `Rotation`.
     */
    Point(au::QuantityD<au::Meters> magnitude, const Rotation& rotation) noexcept;

    /**
     * @brief X coordinate.
     *
     * @return X component (meters).
     */
    au::QuantityD<au::Meters> X() const noexcept;

    /**
     * @brief Y coordinate.
     *
     * @return Y component (meters).
     */
    au::QuantityD<au::Meters> Y() const noexcept;

    /**
     * @brief Vector addition.
     *
     * @param rhs Point to add.
     * @return (x + rhs.x, y + rhs.y).
     */
    Point operator+(const Point& rhs) const noexcept;

    /**
     * @brief Vector subtraction.
     *
     * @param rhs Point to subtract.
     * @return (x - rhs.x, y - rhs.y).
     */
    Point operator-(const Point& rhs) const noexcept;

    /**
     * @brief Unary negation.
     *
     * @return (-x, -y).
     */
    Point operator-() const noexcept;

    /**
     * @brief Scales both components by a unitless scalar.
     *
     * @param scalar Multiplier.
     * @return (x * scalar, y * scalar).
     */
    Point operator*(double scalar) const noexcept;

    /**
     * @brief Divides both components by a unitless scalar.
     *
     * Equivalent to multiplying by `1.0 / scalar`.
     *
     * @param scalar Divisor.
     * @return (x / scalar, y / scalar).
     */
    Point operator/(double scalar) const noexcept;

    /**
     * @brief Direction (heading) of the vector in radians.
     *
     * Returns `atan2(y, x)`. If the point is the origin, returns 0 rad.
     *
     * @return Angle in radians.
     */
    au::QuantityD<au::Radians> theta() const noexcept;

    /**
     * @brief Magnitude (Euclidean norm).
     *
     * @return `hypot(x, y)` in meters.
     */
    au::QuantityD<au::Meters> mag() const noexcept;

    /**
     * @brief Euclidean distance to another point.
     *
     * @param rhs Other point.
     * @return Distance in meters.
     */
    au::QuantityD<au::Meters> distTo(const Point& rhs) const noexcept;

    /**
     * @brief Signed smallest-angle direction from this point’s heading to another’s.
     *
     * Computes `wrap_to_(-π,π](rhs.theta() - theta())`.
     *
     * @param rhs Other point.
     * @return Angle difference in radians.
     */
    au::QuantityD<au::Radians> angleTo(const Point& rhs) const noexcept;

    /**
     * @brief Dot (inner) product.
     *
     * @param rhs Other vector.
     * @return x*rhs.x + y*rhs.y (units: m²).
     */
    au::QuantityD<au::Squared<au::Meters>> dot(const Point& rhs) const noexcept;

    /**
     * @brief 2D wedge (scalar cross) product.
     *
     * Equivalent to the signed z-component of the 3D cross product of
     * (x, y, 0) × (rhs.x, rhs.y, 0): `x*rhs.y - y*rhs.x`.
     *
     * Positive if `rhs` is counter-clockwise from this vector.
     *
     * @param rhs Other vector.
     * @return Signed area (units: m²).
     */
    au::QuantityD<au::Squared<au::Meters>> wedge(const Point& rhs) const noexcept;

    /**
     * @brief Projection of this vector onto another.
     *
     * Returns the component of `*this` along `rhs`:
     * `proj = rhs * (dot(rhs) / rhs.dot(rhs))`.
     *
     * @requires rhs.mag() > 0
     *
     * @param rhs Direction to project onto.
     * @return Projection vector in meters.
     */
    Point project(const Point& rhs) const noexcept;

    /**
     * @brief Rotates this point by a `Rotation` about the origin.
     *
     * Applies the standard 2×2 rotation matrix:
     *
     * @param rhs Rotation to apply.
     * @return Rotated point.
     */
    Point rotateBy(const Rotation& rhs) const noexcept;

    /**
     * @brief Approximate equality in Euclidean metric.
     *
     * @param rhs Other point.
     * @return `true` if `distTo(rhs) ≤ 1e-9 m`, otherwise `false`.
     */
    bool isApprox(const Point& rhs) const noexcept;

    private:
    au::QuantityD<au::Meters> x = au::ZERO;
    au::QuantityD<au::Meters> y = au::ZERO;
};

/**
 * @brief Circumradius of the triangle defined by three points.
 *
 * Uses the formula R = abc/4A, where `a = |BC|`, `b = |AC|`, `c = |AB|`,
 * and `A` is the triangle area (half the absolute wedge of two sides).
 *
 * If the points are collinear (`A = 0`), returns 0 m.
 *
 * @param A Vertex A.
 * @param B Vertex B.
 * @param C Vertex C.
 * @return Circumradius in meters.
 */
au::QuantityD<au::Meters> circumradius(const Point& A, const Point& B, const Point& C) noexcept;


/**
 * @brief Intersection of a circle with a line segment.
 *
 * Solves for `t` in `[0, 1]` where the parametric segment
 * `P(t) = start + t * (end - start)` intersects the circle of `center` and `radius`.
 *
 * If two intersections lie on the segment, the larger parameter (farther along the
 * segment, denoted `t2`) is returned; otherwise the single valid root is returned.
 * If no intersection lies on the segment, returns `std::nullopt`.
 *
 * @param start Segment start point.
 * @param end Segment end point.
 * @param center Circle center.
 * @param radius Circle radius (meters).
 * @return `t` in `[0, 1]` for the intersection point, or `std::nullopt`.
 */
std::optional<double> circleLineIntersection(const Point& start, const Point& end, const Point& center,
                                             au::QuantityD<au::Meters> radius) noexcept;

} // namespace rz
