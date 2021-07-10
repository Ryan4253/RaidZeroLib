#include "Transform2D.hpp"
#include "Pose2d.hpp"

namespace lib4253{

Transform2D::Transform2D(const Pose2D& initial, const Pose2D& final){
    translation = (final.getTranslation() - initial.getTranslation()).rotateBy(-initial.getRotation());

    rotation = final.getRotation() - initial.getRotation();
}

Transform2D::Transform2D(const Translation2D& iTranslation, const Rotation2D& iRotation){
    translation = iTranslation;
    rotation = iRotation;
}

const Translation2D& Transform2D::getTranslation() const{
    return translation;
}

const Rotation2D& Transform2D::getRotation() const{
    return rotation;
}

okapi::QLength Transform2D::getX() const{
    return translation.getX();
}

okapi::QLength Transform2D::getY() const{
    return translation.getY();
}

bool Transform2D::operator==(const Transform2D& rhs) const{
    return translation == rhs.translation && rotation == rhs.rotation;
}

bool Transform2D::operator!=(const Transform2D& rhs) const{
    return !operator==(rhs);
}

Transform2D Transform2D::operator*(double scalar) const{
    return Transform2D(translation * scalar, rotation * scalar);
}

Transform2D Transform2D::inverse() const{
    return Transform2D{(-translation).rotateBy(-rotation), -rotation};
}

}

