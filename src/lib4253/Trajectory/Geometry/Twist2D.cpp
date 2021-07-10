#include "Twist2D.hpp"
namespace lib4253{

Twist2D::Twist2D(const okapi::QLength& x, const okapi::QLength& y, const okapi::QAngle& theta){
    dx = x, dy = y, dtheta = theta;
}


bool Twist2D::operator==(const Twist2D& rhs) const{
    return abs(dx - rhs.dx) < 1E-9 * okapi::meter &&
           abs(dy - rhs.dy) < 1E-9 * okapi::meter &&
           abs(dtheta - rhs.dtheta) < 1E-9 * okapi::radian;
}

bool Twist2D::operator!=(const Twist2D& rhs) const{
    return !operator==(rhs);
}

}