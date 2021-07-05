#pragma once
#include "Transform2D.hpp"
#include "Pose2d.hpp"

namespace lib4253{

Transform2D::Transform2D(Pose2D initial, Pose2D final){
    translation = (final.getTranslation() - initial.getTranslation()).rotateBy(-initial.getRotation());

    rotation = final.getRotation() - initial.getRotation();
}

Transform2D::Transform2D(Translation2D iTranslation, Rotation2D iRotation){

}

const Transform2D& Transform2D::getTranslation() const{

}

const Rotation2D& Transform2D::getRotation() const{

}

okapi::QLength Transform2D::getX() const{

}

okapi::QLength Transform2D::getY() const{

}

Transform2D Transform2D::Inverse() const{

}

Transform2D Transform2D::operator*(double scalar) const{

}

bool Transform2D::operator==(const Transform2D& other) const{

}

bool Transform2D::operator!=(const Transform2D& other) const{

}

}

