#pragma once
#include "RaidZeroLib/api/Pathing/CubicBezier.hpp"

namespace rz {

/**
 * @brief Piecewise cubic Bézier path over t ∈ [0, 1] with segment-uniform parameterization.
 *
 * Builds a chain of cubic Bézier segments from consecutive control “knots.”
 * The global parameter @p t is mapped uniformly by segment: if there are N segments,
 * segment k occupies t ∈ [k/N, (k+1)/N], and the segment-local parameter is
 * `fractionalIndex = N * t − k` in [0, 1].
 *
 * **Continuity:** The curve is C¹ guaranteed continuous, C² is up to the user's choice of control points
 *
 * @see CubicBezier
 */
class PiecewiseCubicBezier : public ParametricPath {
    public:
    /**
     * @brief Constructs a piecewise cubic path from consecutive control knots.
     *
     * For a list of M control points, creates M-1 cubic segments:
     * segment i uses knots `knots[i]` (start) and `knots[i+1]` (end).
     *
     * @param knots Non-empty initializer list of control knots (size ≥ 2).
     * @pre `knots.size() >= 2` to form at least one segment.
     * @note Parameterization is uniform per segment, not by arclength. For
     *       arclength-uniform sampling, use `ParametricPath::toDiscrete` or
     *       `ParametricPath::stepT`.
     */
    explicit PiecewiseCubicBezier(std::initializer_list<CubicBezier::Knot> knots);

    /**
     * @brief Position on the path at global parameter @p t.
     *
     * Maps @p t to the owning segment and evaluates that cubic at its local parameter.
     *
     * @param t Dimensionless global parameter (clamped conceptually to [0, 1]).
     * @return 2D point in meters.
     *
     * @pre 0 ≤ t ≤ 1.
     */
    Point getPoint(double t) const noexcept override;

    /**
     * @brief First derivative dP/dt at global parameter @p t (w.r.t. global parameter).
     *
     * Internally computes the segment index and local parameter, and returns the
     * cubic’s derivative with respect to its local parameter, scaled by the
     * segment mapping (here, uniform per segment).
     *
     * @param t Dimensionless global parameter.
     * @return Tangent vector (meters).
     *
     * @pre 0 ≤ t ≤ 1.
     */
    Point getVelocity(double t) const noexcept override;
    
    /**
     * @brief Second derivative d²P/dt² at global parameter @p t (w.r.t. global parameter).
     *
     * @param t Dimensionless global parameter.
     * @return Second derivative vector (meters).
     *
     * @pre 0 ≤ t ≤ 1.
     */
    Point getAcceleration(double t) const noexcept override;

    private:
    /**
     * @brief Segment/parameter locator for a global parameter.
     *
     * Splits global @p t into:
     * - `segmentIndex` ∈ {0, …, curves.size()-1},
     * - `fractionalIndex` ∈ [0, 1], the segment-local parameter.
     *
     * Special case: for @p t == 1, returns the last segment with `fractionalIndex == 1`.
     */
    struct Index {
        std::size_t segmentIndex;
        double fractionalIndex;
    };

    /**
     * @brief Computes the segment index and local parameter for @p t.
     *
     * Uses a uniform mapping: if there are N segments, `segmentIndex = floor(N * t)` (with
     * the t==1 case mapped to the last segment), and `fractionalIndex = N * t − segmentIndex`.
     *
     * @param t Dimensionless global parameter (conceptually clamped to [0, 1]).
     * @return `{segmentIndex, fractionalIndex}` as described above.
     *
     * @pre 0 ≤ t ≤ 1.
     */
    Index getIndex(double t) const noexcept;

    std::vector<CubicBezier> curves;
};

} // namespace rz
