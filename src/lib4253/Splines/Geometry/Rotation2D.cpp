#include "Rotation2D.hpp"
namespace lib4253{

Rotation2D::Rotation2D(const okapi::QAngle& val){
    value = val;
    sine = (sin(value)).convert(okapi::number);
    cosine = (cos(value)).convert(okapi::number);
}

Rotation2D::Rotation2D(const okapi::QLength& x, const okapi::QLength& y){
    const auto magnitude = hypot(x, y);
    if (magnitude > 1e-6 * okapi::inch) {
        sine = (y / magnitude).convert(okapi::number);
        cosine = (x / magnitude).convert(okapi::number);
    } else {
        sine = 0.0;
        cosine = 1.0;
    }
    value = std::atan2(sine, cosine) * okapi::radian;
}

Rotation2D::Rotation2D(const double& x, const double& y){
    Rotation2D(x * okapi::meter, y * okapi::meter);
}

okapi::QAngle Rotation2D::getVal() const{
    return value;
}

double Rotation2D::getSin() const{
    return sine;
}

double Rotation2D::getCos() const{
    return cosine;
}

double Rotation2D::getTan() const{
    return sine / cosine;
}

Rotation2D Rotation2D::operator+(const Rotation2D& rhs) const{
     return rotateBy(rhs); 
}

Rotation2D Rotation2D::operator-(const Rotation2D& rhs) const{
    return *this + -rhs;
}

Rotation2D Rotation2D::operator-() const{
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

Rotation2D Rotation2D::rotateBy(const Rotation2D& other) const{
    return {(cosine * other.cosine - sine * other.sine) * okapi::meter, (cosine * other.sine + sine * other.cosine) * okapi::meter};
}
}