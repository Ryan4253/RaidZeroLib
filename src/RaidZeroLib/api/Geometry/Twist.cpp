#include "RaidZeroLib/api/Geometry/Twist.hpp"
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include "RaidZeroLib/api/Geometry/Rotation.hpp"
#include "RaidZeroLib/api/Utility/Math.hpp"

namespace rz {

Twist::Twist(au::QuantityD<au::Meters> dX, au::QuantityD<au::Meters> dY, au::QuantityD<au::Radians> dTheta) noexcept 
    : dx(dX), dy(dY), dtheta(constrainAngle180(dTheta)) {}

au::QuantityD<au::Meters> Twist::dX() const noexcept{
    return dx;
}

au::QuantityD<au::Meters> Twist::dY() const noexcept {
    return dy;
}

au::QuantityD<au::Radians> Twist::dTheta() const noexcept {
    return dtheta;
}

bool Twist::isApprox(const Twist& rhs) const noexcept {
    const Point point(dx, dy);
    const Point rhsPoint(rhs.dx, rhs.dy);
    const Rotation rotation(dtheta);
    const Rotation rhsRotation(rhs.dtheta);

    return point.isApprox(rhsPoint) && rotation.isApprox(rhsRotation);
}

} // namespace rz