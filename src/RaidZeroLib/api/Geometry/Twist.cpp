#include "RaidZeroLib/api/Geometry/Twist.hpp"

namespace rz {

Twist::Twist(QLength dX, QLength dY, QAngle dTheta) : dx(dX), dy(dY), dtheta(dTheta) {
}

Twist::Twist(const Twist& rhs) : dx(rhs.dx), dy(rhs.dy), dtheta(rhs.dtheta) {
}

QLength Twist::dX() const {
    return dx;
}

QLength Twist::dY() const {
    return dy;
}

QAngle Twist::dTheta() const {
    return dtheta;
}

bool Twist::operator==(const Twist& rhs) const {
    return abs(dx - rhs.dx) < 1E-9 * okapi::meter && abs(dy - rhs.dy) < 1E-9 * okapi::meter &&
           abs(dtheta - rhs.dtheta) < 1E-9 * okapi::radian;
}

bool Twist::operator!=(const Twist& rhs) const {
    return !operator==(rhs);
}

void Twist::operator=(const Twist& rhs) {
    dx = rhs.dX();
    dy = rhs.dY();
    dtheta = rhs.dTheta();
}

} // namespace rz