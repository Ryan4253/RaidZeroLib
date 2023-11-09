#include "RaidZeroLib/api/Geometry/Transform.hpp" 

namespace rz{

Transform::Transform(const Pose& iInitial, const Pose& iFinal){
    translation = (iFinal.getTranslation() - iInitial.getTranslation()).rotateBy(-iInitial.getRotation());
    rotation = iFinal.getRotation() - iInitial.getRotation();
}

Transform::Transform(const Translation& iTranslation, const Rotation& iRotation) : translation(iTranslation), rotation(iRotation){}

const Translation& Transform::getTranslation() const{
    return translation;
}

const Rotation& Transform::getRotation() const{
    return rotation;
}

okapi::QLength Transform::X() const{
    return translation.X();
}

okapi::QLength Transform::Y() const{
    return translation.Y();
}

QAngle Transform::Theta() const{
    return rotation.Theta();
}

Transform Transform::operator+(const Transform& rhs) const{
    return Transform(Pose(), Pose().transformBy(*this).transformBy(rhs));
}

Transform Transform::operator*(double scalar) const{
    return Transform(translation * scalar, rotation * scalar);
}

Transform Transform::operator/(double scalar) const{
    return *this * (1.0 / scalar);
}

bool Transform::operator==(const Transform& rhs) const{
    return translation == rhs.translation && rotation == rhs.rotation;
}

bool Transform::operator!=(const Transform& rhs) const{
    return !operator==(rhs);
}

void Transform::operator=(const Transform& rhs){
    translation = rhs.getTranslation();
    rotation = rhs.getRotation();
}

Transform Transform::inverse() const{
    return Transform((-translation).rotateBy(-rotation), -rotation);
}

}

