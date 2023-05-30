#include "Rotation.hpp"
namespace rz{

Rotation::Rotation(QAngle iTheta){
    theta = constrainAngle180(iTheta);
    sine = sin(iTheta).convert(number);
    cosine = cos(iTheta).convert(number);
}

Rotation::Rotation(QLength iX, QLength iY){
    const auto magnitude = hypot(iX, iY);
    if (magnitude > 1e-6 * meter) {
        sine = (iY / magnitude).convert(number);
        cosine = (iX / magnitude).convert(number);
    } else {
        sine = 0.0;
        cosine = 1.0;
    }
    theta = std::atan2(sine, cosine) * radian;
}

Rotation::Rotation(double iX, double iY){
    Rotation(iX * meter, iY * meter);
}

QAngle Rotation::Theta() const{
    return theta;
}

double Rotation::Sin() const{
    return sine;
}

double Rotation::Cos() const{
    return cosine;
}

double Rotation::Tan() const{
    return sine / cosine;
}

Rotation Rotation::operator+(const Rotation& rhs) const{
     return rotateBy(rhs); 
}

Rotation Rotation::operator-(const Rotation& rhs) const{
    return *this + -rhs;
}

Rotation Rotation::operator-() const{
    return *this * -1;
}   

Rotation Rotation::operator*(double scalar) const{
    return Rotation(theta * scalar);
}

Rotation Rotation::operator/(double scalar) const{
    return *this * (1/scalar);
}

bool Rotation::operator==(const Rotation& rhs) const{
    return std::hypot(cosine - rhs.cosine, sine - rhs.sine) < 1E-9;
}

bool Rotation::operator!=(const Rotation& rhs) const{
    return !(*this == rhs);
}

void Rotation::operator=(const Rotation& rhs){
    theta = rhs.theta;
    sine = rhs.sine;
    cosine = rhs.cosine;
}

Rotation Rotation::rotateBy(const Rotation& rhs) const{
    return Rotation((cosine * rhs.cosine - sine * rhs.sine) * meter, (cosine * rhs.sine + sine * rhs.cosine) * meter);
}
}