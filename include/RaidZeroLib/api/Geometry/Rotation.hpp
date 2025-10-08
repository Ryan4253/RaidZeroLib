#pragma once
#include "au/au.hpp"

namespace rz {

/**
 * @brief Immutable planar rotation about the Z axis.
 *
 * `Rotation` represents an angle in radians (right-handed, counter-clockwise positive)
 * and provides trigonometric accessors, arithmetic, and comparison helpers.
 *
 * All angles are internally normalized to (-180, 180] degrees so that equivalent
 * rotations share a canonical representation. This guarantees stable arithmetic
 * and comparisons across operations.
 */
class Rotation {
    public:
    /**
     * @brief Constructs a zero rotation.
     *
     * Equivalent to `Rotation(au::ZERO)`; the internal angle is 0 rad.
     */
    constexpr Rotation() noexcept = default;

    /**
     * @brief Constructs from an angle.
     *
     * @param theta the angle.
     */
    explicit Rotation(au::QuantityD<au::Radians> theta) noexcept;

    /**
     * @brief Constructs from a Cartesian vector (x, y).
     *
     * Computes `atan2(y, x)` to obtain the heading of the vector. If both `x` and `y`
     * are zero, the resulting rotation is defined as zero.
     *
     * @param x X component (length units).
     * @param y Y component (length units).
     */
    Rotation(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y) noexcept;

    /**
     * @brief Constructs from a Cartesian vector (x, y).
     *
     * Computes `atan2(y, x)` to obtain the heading of the vector. If both `x` and `y`
     * are zero, the resulting rotation is defined as zero.
     *
     * @param x X component (unitless).
     * @param y Y component (unitless).
     */
    Rotation(double x, double y) noexcept;

    /**
     * @brief Returns the normalized angle in radians.
     *
     * @return Angle in radians.
     */
    au::QuantityD<au::Radians> Theta() const noexcept;

    /**
     * @brief Sine of the rotation.
     *
     * @return `sin(theta)` as a double.
     */
    double Sin() const noexcept;

    /**
     * @brief Cosine of the rotation.
     *
     * @return `cos(theta)` as a double.
     */
    double Cos() const noexcept;

    /**
     * @brief Tangent of the rotation.
     *
     * @return `tan(theta)` as a double.
     */
    double Tan() const noexcept;

    /**
     * @brief Angle addition.
     *
     * @param rhs Rotation to add.
     * @return New rotation equal to `this->theta + rhs.theta`, normalized.
     */
    Rotation operator+(const Rotation& rhs) const noexcept;

    /**
     * @brief Angle subtraction.
     *
     * @param rhs Rotation to subtract.
     * @return New rotation equal to `this->theta - rhs.theta`, normalized.
     */
    Rotation operator-(const Rotation& rhs) const noexcept;

    /**
     * @brief Unary negation.
     *
     * @return New rotation equal to `-theta`, normalized.
     */
    Rotation operator-() const noexcept;

    /**
     * @brief Multiplies the angle by a scalar.
     *
     * @param scalar Multiplication applied to the angle.
     * @return New rotation with angle `theta * scalar`, normalized.
     */
    Rotation operator*(double scalar) const noexcept;

    /**
     * @brief Divides the angle by a scalar.
     *
     * Equivalent to multiplying by `1.0 / scalar`.
     *
     * @param scalar Divisor applied to the angle.
     * @return New rotation with angle `theta / scalar`, normalized.
     */
    Rotation operator/(double scalar) const noexcept;

    /**
     * @brief Approximate equality check with angular tolerance.
     *
     * Computes the normalized minimum signed difference between the two rotations
     * and compares its absolute value to 1e-9 radians.
     *
     * @param rhs Rotation to compare with.
     * @return `true` if the two rotation within 1e-9 radians, otherwise `false`.
     */
    bool isApprox(const Rotation& rhs) const noexcept;

    private:
    au::QuantityD<au::Radians> theta = au::ZERO;
};

} // namespace rz
