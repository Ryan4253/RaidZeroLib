#pragma once
#include "au/au.hpp"

namespace rz {

/**
 * @brief Differential motion (twist) in SE(2) space.
 *
 * `Twist` represents an infinitesimal rigid-body motion consisting of translational
 * displacements `dX`, `dY` (in meters) and a rotational displacement `dTheta`
 * (in radians). It is typically used with the SE(2) exponential and logarithmic
 * maps (`Pose::exp`, `Pose::log`) to integrate or extract small motions between poses.
 *
 * All angular quantities are normalized to (-180, 180] degrees.
 */
class Twist {
    public:
    /**
     * @brief Constructs a twist with translational and rotational components.
     *
     * @param dx Translational displacement along the X-axis (meters).
     * @param dy Translational displacement along the Y-axis (meters).
     * @param dtheta Rotational displacement (radians).
     */
    Twist(au::QuantityD<au::Meters> dx, au::QuantityD<au::Meters> dy, au::QuantityD<au::Radians> dtheta) noexcept;

    /**
     * @brief Differential translation along X.
     *
     * @return Displacement in meters.
     */
    au::QuantityD<au::Meters> dX() const noexcept;

    /**
     * @brief Differential translation along Y.
     *
     * @return Displacement in meters.
     */
    au::QuantityD<au::Meters> dY() const noexcept;

    /**
     * @brief Differential rotation about +Z (right-handed, CCW positive).
     *
     * @return Angular displacement in radians, normalized to (-π, π].
     */
    au::QuantityD<au::Radians> dTheta() const noexcept;

    /**
     * @brief Approximate equality check with tight tolerance.
     *
     * Compares translational and rotational components with a tolerance
     * of ~1e-9 m and ~1e-9 rad respectively, using `Point::isApprox`
     * and `Rotation::isApprox`.
     *
     * @param rhs Twist to compare with.
     * @return `true` if both translations and rotation are approximately equal.
     */
    bool isApprox(const Twist& rhs) const noexcept;

    private:
    au::QuantityD<au::Meters> dx = au::ZERO;
    au::QuantityD<au::Meters> dy = au::ZERO;
    au::QuantityD<au::Radians> dtheta = au::ZERO;
};

} // namespace rz
