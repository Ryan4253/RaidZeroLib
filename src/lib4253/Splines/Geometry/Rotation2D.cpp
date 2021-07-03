#include "lib4253/Splines/Geometry/Rotation2D.hpp"
using namespace okapi;
namespace lib4253{

Rotation2D::Rotation2D(QAngle val){
    value = val;
    sine = (sin(value)).convert(number);
    cosine = (cos(value)).convert(number);
}

Rotation2D::Rotation2D(QLength x, QLength y){
    const auto magnitude = hypot(x, y);
    if (magnitude > 1e-6 * inch) {
        sine = (y / magnitude).convert(number);
        cosine = (x / magnitude).convert(number);
    } else {
        sine = 0.0;
        cosine = 1.0;
    }
    value = std::atan2(sine, cosine) * radian;
}

QAngle Rotation2D::getVal() const{
    return value;
}

double Rotation2D::getSin() const{
    return sine;
}

double Rotation2D::getCos() const{
    return cosine;
}

double Rotation2D::getTan() const{
    return sine/cosine;
}

Rotation2D Rotation2D::operator+(const Rotation2D& rhs) const{
     return rotateBy(rhs); 
}

Rotation2D Rotation2D::operator-(const Rotation2D& rhs) const{
    return *this + -rhs;
}

Rotation2D Rotation2D::operator-()  const{
    return Rotation2D(value * -1);
}   

Rotation2D Rotation2D::operator*(const double& scalar) const{
    return Rotation2D(value * scalar);
}

bool Rotation2D::operator==(const Rotation2D& rhs) const{
    return std::hypot(cosine - rhs.cosine, sine - rhs.sine) < 1E-9;
}

bool Rotation2D::operator!=(const Rotation2D& rhs) const{
    return !operator==(rhs);
}

Rotation2D Rotation2D::rotateBy(const Rotation2D& rhs) const{
    return {(cosine * rhs.cosine - sine * rhs.sine) * inch, (cosine * rhs.sine + sine * rhs.cosine) * inch};
}
}