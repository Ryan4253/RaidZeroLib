#include "Pose.hpp"
namespace lib4253{

Pose::Pose(const Translation& iTranslation, const Rotation& iRotation){
    translation = iTranslation;
    rotation = iRotation;
}

Pose::Pose(QLength iX, QLength iY, const Rotation& iRotation)
    : translation(iX, iY), rotation(iRotation){}

Pose::Pose(const OdomState& iState){
    translation = Translation(iState.x, -1*iState.y);
    rotation = Rotation(iState.theta);
}

const Translation& Pose::getTranslation() const{
    return translation;
}

const Rotation& Pose::getRotation() const{
    return rotation;
}

QLength Pose::X() const{
    return translation.X();
}

QLength Pose::Y() const{
    return translation.Y();
}

QAngle Pose::Theta() const{
    return rotation.Theta();
}

Pose Pose::operator+(const Transform& rhs) const{
    return transformBy(rhs);
}

Transform Pose::operator-(const Pose& rhs) const{
    const Pose pose = this->relativeTo(rhs);
    return Transform(pose.getTranslation(), pose.getRotation());
}

bool Pose::operator==(const Pose& rhs) const{
    return translation == rhs.translation && rotation == rhs.rotation;
}

bool Pose::operator!=(const Pose& rhs) const{
    return !operator==(rhs);
}

void Pose::operator=(const Pose& rhs){
    translation = rhs.getTranslation();
    rotation = rhs.getRotation();
}

Point Pose::closestTo(const Point& rhs) const{
    /*
    Point current = translation;
    Point heading(sin(rotation.getVal()), cos(rotation.getVal()));
    Point n = heading.normalize();
    Point v = target-translation;
    double d = n*v  ;
    return (current)+((n*d));
    */
    Point diff = rhs - translation;
    QLength mag = diff.mag();
    QAngle angle = (*this).angleTo(diff);
    QLength inc = mag * cos(angle);
    return translation + Translation(inc * cos(angle), inc * sin(angle));
}

Pose Pose::transformBy(const Transform& rhs) const{
    return {translation + (rhs.getTranslation().rotateBy(rotation)), rotation + rhs.getRotation()};
}

Pose Pose::relativeTo(const Pose& rhs) const{
    const Transform transform{rhs, *this};
    return {transform.getTranslation(), transform.getRotation()};
}

QAngle Pose::angleTo(const Point& rhs) const{
    return rotation.Theta() - (rhs-translation).Theta();
}

Pose Pose::exp(const Twist& rhs) const{
    const QLength dx = rhs.dX();
    const QLength dy = rhs.dY();
    const double dtheta = rhs.dTheta().convert(radian);

    const auto sinTheta = std::sin(dtheta);
    const auto cosTheta = std::cos(dtheta);

    double s, c;
    if(std::abs(dtheta) < 1E-9){
        s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
        c = 0.5 * dtheta;
    }
    else{
        s = sinTheta / dtheta;
        c = (1 - cosTheta) / dtheta;
    }

    const Transform transform{Translation{dx * s - dy * c, dx * c + dy * s}, Rotation{cosTheta * meter, sinTheta * meter}};

    return *this + transform;
}

Twist Pose::log(const Pose& rhs) const{
    const Pose transform = rhs.relativeTo(*this);
    const double dtheta = transform.getRotation().Theta().convert(radian);
    const double halfDtheta = dtheta / 2.0;

    const double cosMinusOne = transform.getRotation().Cos() - 1;

    double halfThetaByTanOfHalfDtheta;

    if (std::abs(cosMinusOne) < 1E-9) {
      halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } 
    else {
        halfThetaByTanOfHalfDtheta =
        -(halfDtheta * transform.getRotation().Sin()) / cosMinusOne;
    }

    const Translation translationPart =
    transform.getTranslation().rotateBy({halfThetaByTanOfHalfDtheta, -halfDtheta}) * std::hypot(halfThetaByTanOfHalfDtheta, halfDtheta);

    return {translationPart.X(), translationPart.Y(), (dtheta * radian)};
}

}