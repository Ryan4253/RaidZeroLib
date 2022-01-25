#pragma once
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace lib4253{
using namespace okapi;

class Twist{
    public:

    Twist(QLength iX, QLength iY, QAngle iTheta);

    ~Twist() = default;

	QLength dX() const;

	QLength dY() const;

	QAngle dTheta() const;

	/**
	 * Checks equality between this Twist and another object.
	 *
	 * @param rhs The other object.
	 * @return Whether the two objects are equal.
	 */
	bool operator==(const Twist& rhs) const;

	/**
	 * Checks inequality between this Twist and another object.
	 *
	 * @param rhs The other object.
	 * @return Whether the two objects are not equal.
	 */
	bool operator!=(const Twist& rhs) const;

	void operator=(const Twist& rhs);

	private:
	/**
     * Linear "dx" component
     */
    QLength dx = 0_m;

    /**
     * Linear "dy" component
     */
    QLength dy = 0_m;

    /**
     * Angular "dtheta" component (radians)
     */
    QAngle dtheta = 0_rad;
};

}