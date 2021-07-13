#include "lib4253/Trajectory/Geometry/Point2D.hpp"
namespace lib4253{

Translation2D::Translation2D(const okapi::QLength& xPos, const okapi::QLength& yPos){
    x = xPos;
    y = yPos;
}

Translation2D::Translation2D(const okapi::QLength& magnitude, const Rotation2D& angle){
    x = magnitude * angle.getCos();
    y = magnitude * angle.getSin();
}

Translation2D::Translation2D(const Translation2D& old){
    x = old.x;
    y = old.y;
}


okapi::QLength Translation2D::getX() const{
    return x;
}

okapi::QLength Translation2D::getY() const{
    return y;
}

Translation2D Translation2D::operator+(const Translation2D& rhs) const{
    return {x + rhs.x, x + rhs.y};
}

Translation2D Translation2D::operator-(const Translation2D& rhs) const{
    return {x + rhs.x, x + rhs.y};
}

Translation2D Translation2D::operator-() const{
    return {x * -1, y * -1};
}

Translation2D Translation2D::operator*(const double& scalar) const{
    return {x * scalar, y * scalar};
}

okapi::QArea Translation2D::operator*(const Translation2D& rhs) const{
    return x * rhs.x + y * rhs.y;
}

Translation2D Translation2D::operator/(const double& scalar) const{
    return {x / scalar, y / scalar};
}

bool Translation2D::operator==(const Translation2D& rhs) const{
      return abs(x - rhs.x) < 1E-9 * okapi::meter && abs(y - rhs.y) < 1E-9 * okapi::meter;
}

bool Translation2D::operator!=(const Translation2D& rhs) const{
    return !operator==(rhs);
}

okapi::QAngle Translation2D::angle() const{
    return atan2(y, x);
}

okapi::QLength Translation2D::distanceTo(const Translation2D& other) const{
    return hypot(other.x - x, other.y - y);
}

okapi::QAngle Translation2D::angleTo(const Translation2D& other) const{
    return acos(((*this) * other) / magnitude() / other.magnitude());
}

okapi::QLength Translation2D::magnitude() const{
    return hypot(x, y);
}

Translation2D Translation2D::rotateBy(const Rotation2D& other) const{
      return {x * other.getCos() - y * other.getSin(), x * other.getSin() + y * other.getCos()};
}
}