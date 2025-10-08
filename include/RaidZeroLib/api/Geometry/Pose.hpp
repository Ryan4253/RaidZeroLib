#pragma once
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include "RaidZeroLib/api/Geometry/Rotation.hpp"

namespace okapi {
class OdomState;
} // namespace okapi

namespace rz {

class Transform;
class Twist;

/**
 * @brief Rigid-body pose in SE(2) with unit-safe position and rotation.
 *
 * `Pose` stores a 2D position (`Point`, meters) and a heading (`Rotation`, radians).
 * It provides composition relative to a `Transform`, relative transforms between poses,
 * and SE(2) exponential / logarithmic maps for integrating small twists or extracting
 * differential motion.
 *
 * Headings are right-handed, counter-clockwise about +Z, normalized to (-180, 180] degrees.
 */
class Pose {
    public:
    /// @brief Constructs the identity pose at the origin with zero heading.
    constexpr Pose() noexcept = default;

    /**
     * @brief Constructs from a point and rotation.
     *
     * @param point Position in meters.
     * @param rotation Heading (normalized to (-π, π]).
     */
    Pose(const Point& point, const Rotation& rotation) noexcept;

    /**
     * @brief Constructs from Cartesian components and a rotation.
     *
     * @param x X coordinate (meters).
     * @param y Y coordinate (meters).
     * @param rotation Heading (normalized to (-π, π]).
     */
    Pose(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y, const Rotation& rotation) noexcept;

    /**
     * @brief Constructs from Cartesian components and an angle.
     *
     * @param x X coordinate (meters).
     * @param y Y coordinate (meters).
     * @param angle Heading angle (radians), normalized to (-π, π].
     */
    Pose(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y, au::QuantityD<au::Radians> angle) noexcept;

    /**
     * @brief Constructs from an OkapiLib odometry state in "frame transformation" mode
     *
     * Converts Okapi’s coordinate frame to this library’s frame:
     * `x = state.x (m)`, `y = -state.y (m)`, `theta = -state.theta (rad)`.
     * The negations reflect axis/heading conventions.
     *
     * @param state Okapi odometry state.
     */
    Pose(const okapi::OdomState& state) noexcept;

    /// @brief Access the position.
    const Point& getPoint() const noexcept;

    /// @brief Access the heading.
    const Rotation& getRotation() const noexcept;

    /// @brief X position (meters).
    au::QuantityD<au::Meters> X() const noexcept;

    /// @brief Y position (meters).
    au::QuantityD<au::Meters> Y() const noexcept;

    /// @brief Heading angle (radians, in (-π, π]).
    au::QuantityD<au::Radians> Theta() const noexcept;

    /**
     * @brief Applies a rigid transform to this pose (pose ⊕ transform).
     *
     * Equivalent to: rotate the transform’s translation by this pose’s heading,
     * then translate, and add the headings.
     *
     * @param rhs Transform to apply in this pose’s local frame.
     * @return Resulting pose.
     */
    Pose transformBy(const Transform& rhs) const noexcept;

    /**
     * @brief Computes the transform that takes `rhs` into `*this` (rhs ⊖ this).
     *
     * @param rhs Reference pose to measure from.
     * @return Transform such that `rhs.transformBy(result) == *this`.
     */
    Transform relativeTo(const Pose& rhs) const noexcept;

    /**
     * @brief SE(2) exponential map: integrates a body-frame twist.
     *
     * For a small body-frame twist (dX, dY, dTheta), returns
     * `this ⊕ Exp(dX, dY, dTheta)`. Uses a small-angle approximation for
     * |dTheta| < 1e-9 to maintain numerical stability.
     *
     * @param rhs Body-frame twist.
     * @return New pose after applying the twist.
     */
    Pose exp(const Twist& rhs) const noexcept;

    /**
     * @brief SE(2) logarithmic map: body-frame twist from this pose to `rhs`.
     *
     * Computes `Log(transform)`, where `transform = rhs.relativeTo(*this)`.
     * Uses a stable formula with a small-angle branch for |dTheta| < 1e-9.
     *
     * @param rhs Target pose.
     * @return Body-frame twist that approximately maps `*this` to `rhs`.
     */
    Twist log(const Pose& rhs) const noexcept;

    /**
     * @brief Approximate equality of both position and heading.
     *
     * @param rhs Other pose.
     * @return `true` if points are within 1e-9 m and headings within 1e-9 rad.
     */
    bool isApprox(const Pose& rhs) const noexcept;

    private:
    Point point;
    Rotation rotation;
};

/**
 * @brief Signed curvature needed to reach a point from a pose assuming an instantaneous circular arc.
 *
 * Computes curvature = +-2 * dPerp / chord^2, where dPerp is the perpendicular distance 
 * from the point to the pose’s heading line, and the sign is determined by which side 
 * of the heading the point lies on (left positive, right negative). If the point lies 
 * exactly on the heading line, returns 0.
 *
 * @param pose Current pose (defines line and heading).
 * @param point Target point.
 * @return Curvature in 1/meters.
 */
au::QuantityD<au::Inverse<au::Meters>> curvatureToPoint(const Pose& pose, const Point& point) noexcept;

} // namespace rz
