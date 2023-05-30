#include "Transform.hpp"

namespace lib4253{

Transform::Transform(const Pose& initial, const Pose& final){
    translation = (final.getTranslation() - initial.getTranslation()).rotateBy(-initial.getRotation());

    rotation = final.getRotation() - initial.getRotation();
}

Transform::Transform(const Translation& iTranslation, const Rotation& iRotation){
    translation = iTranslation;
    rotation = iRotation;
}

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

Transform Transform::operator*(double scalar) const{
    return Transform(translation * scalar, rotation * scalar);
}

Transform Transform::operator/(double scalar) const{
    return Transform(translation / scalar, rotation / scalar);
}

Transform Transform::inverse() const{
    return Transform{(-translation).rotateBy(-rotation), -rotation};
}

}

