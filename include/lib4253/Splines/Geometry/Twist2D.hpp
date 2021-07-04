#pragma once
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace lib4253{
class Twist2D{
    public:

    Twist2D(const okapi::QLength& x, const okapi::QLength& y, const okapi::QAngle& theta);

    ~Twist2D() = default;

    /**
     * Checks equality between this Twist2d and another object.
     *
     * @param rhs The other object.
     * @return Whether the two objects are equal.
     */
    bool operator==(const Twist2D& rhs) const;

    /**
     * Checks inequality between this Twist2d and another object.
     *
     * @param rhs The other object.
     * @return Whether the two objects are not equal.
     */
    bool operator!=(const Twist2D& rhs) const;

	/**
     * Linear "dx" component
     */
    okapi::QLength dx = 0 * okapi::meter;

    /**
     * Linear "dy" component
     */
    okapi::QLength dy = 0 * okapi::meter;

    /**
     * Angular "dtheta" component (radians)
     */
    okapi::QAngle dtheta = 0 * okapi::radian;
  };

}