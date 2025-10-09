#pragma once
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include "RaidZeroLib/api/Pathing/ParametricPath.hpp"

namespace rz {

/**
 * @brief Cubic Bézier parametric path segment.
 *
 * A `CubicBezier` represents a smooth parametric curve defined by four control points:
 * \f[
 * P(t) = (1-t)^3 c_0 + 3(1-t)^2 t c_1 + 3(1-t)t^2 c_2 + t^3 c_3, \quad t \in [0, 1].
 * \f]
 *
 * Each segment is constructed from two knots, which encode both position
 * and tangent direction/length at their endpoints. The constructor expands those into
 * the four cubic control points:
 * - `c₀ = start.getPoint()`
 * - `c₁ = start.getForwardControl()`
 * - `c₂ = end.getBackwardControl()`
 * - `c₃ = end.getPoint()`
 *
 * **Continuity:** When consecutive segments share the same end/start `Knot`,
 * the resulting chain is automatically C⁰ and C¹ continuous at the join.
 *
 * @see PiecewiseCubicBezier
 */
class CubicBezier : public ParametricPath {
    public:
    /**
     * @brief Control knot defining both position and tangent handle geometry.
     *
     * Each `Knot` specifies a point `(x, y)` in meters, an outgoing tangent
     * direction `theta` in radians, and a tangent handle length `magnitude` in meters.
     * From these, two derived “handles” are computed:
     * - Forward control: `point + magnitude * [cos(theta), sin(theta)]`
     * - Backward control: `point - magnitude * [cos(theta), sin(theta)]`
     *
     * These handles are used when constructing Bézier segments:
     * the *start* control point contributes its **forward** handle,
     * and the *end* control point contributes its **backward** handle.
     */
    class Knot {
        public:
        /**
         * @brief Constructs a control knot with position, tangent angle, and handle length.
         *
         * @param x X coordinate (meters).
         * @param y Y coordinate (meters).
         * @param theta Tangent direction (radians).
         * @param magnitude Length of tangent handle (meters).
         *
         * @pre `magnitude > 0`.
         */
        Knot(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y, 
                     au::QuantityD<au::Radians> theta, au::QuantityD<au::Meters> magnitude) noexcept;

        /// @brief Returns the anchor position (x, y) of the control knot.
        Point getPoint() const noexcept;

        /// @brief Returns the forward (outgoing) control handle in world coordinates.
        Point getForwardControl() const noexcept;

        /// @brief Returns the backward (incoming) control handle in world coordinates.
        Point getBackwardControl() const noexcept;

        private:
        au::QuantityD<au::Meters> x;
        au::QuantityD<au::Meters> y;
        au::QuantityD<au::Radians> theta;
        au::QuantityD<au::Meters> magnitude;
    };

    /**
     * @brief Constructs a cubic Bézier segment between two control knots.
     *
     * The start and end control points define the endpoints and tangents.
     * Internally, the four Bézier control points are expanded as:
     * ```
     * c0 = start.getPoint();
     * c1 = start.getForwardControl();
     * c2 = end.getBackwardControl();
     * c3 = end.getPoint();
     * ```
     *
     * @param start Start control knot.
     * @param end   End control knot.
     */
    CubicBezier(const Knot& start, const Knot& end) noexcept;

    /**
     * @brief Computes the position P(t) along the cubic Bézier.
     *
     * @param t Normalized parameter ∈ [0, 1].
     * @return 2D point (meters).
     */
    Point getPoint(double t) const noexcept override;

    /**
     * @brief Computes the first derivative dP/dt at parameter t.
     *
     * The derivative is given by:
     * \f[
     * P'(t) = 3(1-t)^2 (c_1 - c_0) + 6(1-t)t (c_2 - c_1) + 3t^2 (c_3 - c_2)
     * \f]
     *
     * @param t Normalized parameter ∈ [0, 1].
     * @return Velocity vector (meters per unit t).
     */
    Point getVelocity(double t) const noexcept override;

    /**
     * @brief Computes the second derivative d²P/dt² at parameter t.
     *
     * The expression is:
     * \f[
     * P''(t) = 6(1-t)(c_2 - 2c_1 + c_0) + 6t(c_3 - 2c_2 + c_1)
     * \f]
     *
     * @param t Normalized parameter ∈ [0, 1].
     * @return Acceleration vector (meters per unit t²).
     */
    Point getAcceleration(double t) const noexcept override;

    private:
    Point c0;
    Point c1;
    Point c2;
    Point c3;
};

} // namespace rz
