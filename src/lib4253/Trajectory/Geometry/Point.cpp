#include "Point.hpp"
namespace lib4253{

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

Translation Translation::operator+(const Translation& rhs) const{
    return {x + rhs.x, x + rhs.y};
}

Translation Translation::operator-(const Translation& rhs) const{
    return {x + rhs.x, x + rhs.y};
}

Translation Translation::operator-() const{
    return {x * -1, y * -1};
}

Translation Translation::operator*(double scalar) const{
    return {x * scalar, y * scalar};
}

QArea Translation::operator*(const Translation& rhs) const{
    return x * rhs.x + y * rhs.y;
}

Translation Translation::operator/(double scalar) const{
    return {x / scalar, y / scalar};
}

bool Translation::operator==(const Translation& rhs) const{
      return abs(x - rhs.x) < 1E-9 * meter && abs(y - rhs.y) < 1E-9 * meter;
}

bool Translation::operator!=(const Translation& rhs) const{
    return !operator==(rhs);
}

void Translation::operator=(const Translation& rhs){
    x = rhs.x, y = rhs.y;
}

QAngle Translation::Theta() const{
    return atan2(y, x);
}

QLength Translation::distTo(const Translation& rhs) const{
    return hypot(rhs.x - x, rhs.y - y);
}

QAngle Translation::angleTo(const Translation& rhs) const{
    return acos(((*this) * rhs) / mag() / rhs.mag());
}

QLength Translation::mag() const{
    return hypot(x, y);
}

Translation Translation::rotateBy(const Rotation& rhs) const{
      return {x * rhs.Cos() - y * rhs.Sin(), x * rhs.Sin() + y * rhs.Cos()};
}

void Translation::setX(QLength iX){
    x = iX;
}

void Translation::setY(QLength iY){
    y = iY;
}
}