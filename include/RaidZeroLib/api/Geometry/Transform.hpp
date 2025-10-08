#pragma once
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include "RaidZeroLib/api/Geometry/Rotation.hpp"

namespace rz {

class Pose;

/**
 * @brief Rigid-body transform in SE(2): translation (meters) + rotation (radians).
 *
 * `Transform` represents a body-frame motion: a rotation about +Z (right-handed,
 * CCW positive, normalized to (-180, 180] degrees) and a translation expressed in the
 * transform’s own frame when used compositionally with `Pose`.
 *
 * Key semantics:
 * - Constructing from two poses, `Transform(initial, final)`, produces the body-frame
 *   motion that maps `initial` to `final`: rotate by `-initial.R`, then translate,
 *   with heading delta `final.R - initial.R`.
 * - Composition `A + B` is the SE(2) composition (apply `A`, then `B`).
 * - `inverse()` returns the exact inverse motion.
 */
class Transform {
    public:
    /// @brief Identity transform: zero translation, zero rotation.
    constexpr Transform() noexcept = default;

    /**
     * @brief Body-frame transform from `initial` to `final`.
     *
     * Translation is `(final.p - initial.p)` expressed in the `initial` local frame:
     * `(final.p - initial.p).rotateBy(-initial.R)`. Rotation is `final.R - initial.R`
     * (normalized).
     *
     * @param initial Start pose.
     * @param final   End pose.
     */
    Transform(const Pose& initial, const Pose& final) noexcept;

    /**
     * @brief Constructs directly from a translation and rotation.
     *
     * @param point Translation (meters).
     * @param rotation Rotation (radians, normalized to (-π, π]).
     */
    Transform(const Point& point, const Rotation& rotation) noexcept;

    /// @brief Access the translation component.
    const Point& getPoint() const noexcept;

    /// @brief Access the rotation component.
    const Rotation& getRotation() const noexcept;

    /// @brief X translation (meters).
    au::QuantityD<au::Meters> X() const noexcept;

    /// @brief Y translation (meters).
    au::QuantityD<au::Meters> Y() const noexcept;

    /// @brief Heading delta (radians, in (-π, π]).
    au::QuantityD<au::Radians> Theta() const noexcept;

    /**
     * @brief SE(2) composition (apply this, then rhs).
     *
     * Equivalent to composing via poses: `Pose().transformBy(*this).transformBy(rhs)`
     * and extracting the resulting transform from identity.
     *
     * @param rhs Transform to apply after this one.
     * @return Composed transform.
     */
    Transform operator+(const Transform& rhs) const noexcept;

    /**
     * @brief Unary inverse.
     *
     * Returns the unique transform that composes with `*this` to yield identity.
     * Translation is rotated by the negative rotation and negated accordingly.
     *
     * @return Inverse transform.
     */
    Transform operator-() const noexcept;

    /**
     * @brief Approximate equality on both translation and rotation.
     *
     * Uses `Point::isApprox` (≤ 1e-9 m) and `Rotation::isApprox` (≤ 1e-9 rad).
     *
     * @param rhs Other transform.
     * @return `true` if both components are approximately equal, otherwise `false`.
     */
    bool isApprox(const Transform& rhs) const noexcept;

    private:
    Point point;
    Rotation rotation;
};

} // namespace rz