#pragma once
#include "RaidZeroLib/api/Pathing/DiscretePath.hpp"
#include "au/au.hpp"

namespace rz {


/**
 * @brief Differentiable 2D path over a unit-interval parameter t ∈ [0, 1].
 *
 * Implementers provide position and its first two derivatives with respect to the
 * *dimensionless* path parameter @p t. Higher-level queries—heading, curvature,
 * arclength, parameter stepping by distance, and discretization—are derived here.
 *
 * **Units & semantics**
 * - `getPoint(t)` → position in meters (`Point` stores meter-valued components).
 * - `getVelocity(t)` → dP/dt in meters (derivative w.r.t. parameter, not time).
 * - `getAcceleration(t)` → d²P/dt² in meters (w.r.t. parameter).
 * - `getTheta(t)` → heading of the tangent (radians).
 * - `getCurvature(t)` → signed curvature κ = wedge(dP/dt, d²P/dt²) / |dP/dt|³ (1/meters).
 * - `getLength(a,b)` → ∫_a^b |dP/dt| dt (meters).
 *
 * **Domain**: All functions expect 0 ≤ t ≤ 1 (implementations may clamp internally).
 *
 * **Continuity**: For stable curvature, the path should be at least C¹ and piecewise C².
 */
class ParametricPath {
    public:
    /// @brief Virtual destructor.
    virtual ~ParametricPath() = default;

    /**
     * @brief Position on the path at parameter @p t.
     *
     * @param t Dimensionless parameter, typically in [0, 1].
     * @return 2D point in meters.
     *
     * @pre 0 ≤ t ≤ 1.
     */
    virtual Point getPoint(double t) const noexcept = 0;

    /**
     * @brief First derivative dP/dt at parameter @p t (w.r.t. path parameter).
     *
     * @param t Dimensionless parameter.
     * @return Tangent vector; magnitude has units of meters.
     *
     * @note This is not velocity w.r.t. physical time.
     *
     * @pre 0 ≤ t ≤ 1.
     */
    virtual Point getVelocity(double t) const noexcept = 0;

    /**
     * @brief Second derivative d²P/dt² at parameter @p t (w.r.t. path parameter).
     *
     * @param t Dimensionless parameter.
     * @return Second derivative vector; components in meters.
     *
     * @pre 0 ≤ t ≤ 1.
     */
    virtual Point getAcceleration(double t) const noexcept = 0;

    /**
     * @brief Path heading: orientation of the tangent dP/dt.
     *
     * Equivalent to `atan2(dy/dt, dx/dt)` via `Point::theta()`.
     *
     * @param t Dimensionless parameter.
     * @return Heading angle in radians.
     *
     * @pre 0 ≤ t ≤ 1.
     *
     * @note Returns 0 when |dP/dt| ≈ 0 (stationary/cusp).
     */
    virtual au::QuantityD<au::Radians> getTheta(double t) const noexcept;


    /**
     * @brief Signed curvature κ(t) = wedge(dP/dt, d²P/dt²) / |dP/dt|³.
     *
     * Positive for left-turning, negative for right-turning (right-handed coordinates).
     *
     * @param t Dimensionless parameter.
     * @return Curvature in 1/meters.
     *
     * @pre 0 ≤ t ≤ 1.
     *
     * @note Returns 0 when |dP/dt| is ~0 to avoid division by zero.
     */
    virtual au::QuantityD<au::Inverse<au::Meters>> getCurvature(double t) const noexcept;

    /**
     * @brief Arclength along the path between parameters @p tStart and @p tEnd.
     *
     * Computes s = ∫_{tStart}^{tEnd} |dP/dt| dt using adaptive Gauss–Legendre quadrature.
     * https://en.wikipedia.org/wiki/Adaptive_quadrature
     *
     * @param tStart Start parameter (default 0).
     * @param tEnd   End parameter (default 1).
     * @return Length in meters.
     *
     * @pre 0 ≤ tStart ≤ tEnd ≤ 1.
     */
    virtual au::QuantityD<au::Meters> getLength(double tStart = 0, double tEnd = 1) const noexcept;

    /**
     * @brief Advance the parameter by a target arclength measured along the path.
     *
     * Finds t' ≥ t such that ∫_{t}^{t'} |dP/dt| dt ≈ @p distance. Uses a safeguarded
     * Newton/bisection refinement with bracketing and clamps to t' ≤ 1.
     * https://discourse.julialang.org/t/references-and-implementation-for-safe-hybrid-univariate-newtons-method/87638
     *
     * @param t        Current parameter in [0, 1].
     * @param distance Desired arclength step (meters), s ≥ 0.
     * @return Next parameter t' ∈ [t, 1].
     *
     * @note If @p distance == 0 or t == 1, returns 1.
     */
    virtual double stepT(double t, au::QuantityD<au::Meters> distance) const noexcept;

    virtual DiscretePath toDiscrete(au::QuantityD<au::Meters> distance, bool includeEnd = true) const;
};

} // namespace rz
