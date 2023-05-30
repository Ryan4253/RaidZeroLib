#include "Point.hpp"
namespace rz{

Translation::Translation(QLength iX, QLength iY){
    x = iX;
    y = iY;
}

Translation::Translation(QLength iMag, const Rotation& iAngle){
    x = iMag * iAngle.Cos();
    y = iMag * iAngle.Sin();
}

Translation::Translation(const Translation& rhs){
    x = rhs.x;
    y = rhs.y;
}

QLength Translation::X() const{
    return x;
}

QLength Translation::Y() const{
    return y;
}

void Translation::setX(QLength iX){
    x = iX;
}

void Translation::setY(QLength iY){
    y = iY;
}

Translation Translation::operator+(const Translation& rhs) const{
    return Point(x + rhs.x, x + rhs.y);
}

Translation Translation::operator-(const Translation& rhs) const{
    return -rhs + *this;
}

Translation Translation::operator-() const{
    return *this * -1;
}

Translation Translation::operator*(double scalar) const{
    return Point(x * scalar, y * scalar);
}

Translation Translation::operator/(double scalar) const{
    return *this * (1/scalar);
}

bool Translation::operator==(const Translation& rhs) const{
      return abs(x - rhs.x) < 1E-9 * meter && abs(y - rhs.y) < 1E-9 * meter;
}

bool Translation::operator!=(const Translation& rhs) const{
    return !(*this==rhs);
}

void Translation::operator=(const Translation& rhs){
    x = rhs.x, y = rhs.y;
}

QAngle Translation::theta() const{
    return atan2(y, x);
}

QLength Translation::mag() const{
    return hypot(x, y);
}

QLength Translation::distTo(const Translation& rhs) const{
    return hypot(rhs.x - x, rhs.y - y);
}

QAngle Translation::angleTo(const Translation& rhs) const{
    return theta() - rhs.theta();
}

QArea Translation::dot(const Translation& rhs) const{
    return x * rhs.x + y * rhs.y;
}

QArea Translation::wedge(const Translation& rhs) const{
    return x * rhs.y - y * rhs.x;
}

Translation Translation::rotateBy(const Rotation& rhs) const{
      return {x * rhs.Cos() - y * rhs.Sin(), x * rhs.Sin() + y * rhs.Cos()};
}

}