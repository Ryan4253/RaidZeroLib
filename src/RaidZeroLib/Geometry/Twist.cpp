#include "Twist.hpp"
namespace lib4253{

Twist::Twist(QLength iX, QLength iY, QAngle iTheta){
    dx = iX, dy = iY, dtheta = iTheta;
}

QLength Twist::dX() const{
    return dx;
}

QLength Twist::dY() const{
    return dy;
}

QAngle Twist::dTheta() const{
    return dtheta;
}

bool Twist::operator==(const Twist& rhs) const{
    return abs(dx - rhs.dx) < 1E-9 * okapi::meter &&
           abs(dy - rhs.dy) < 1E-9 * okapi::meter &&
           abs(dtheta - rhs.dtheta) < 1E-9 * okapi::radian;
}

bool Twist::operator!=(const Twist& rhs) const{
    return !operator==(rhs);
}

void Twist::operator=(const Twist& rhs){
    dx = rhs.dX();
    dy = rhs.dY();
    dtheta = rhs.dTheta();
}


}