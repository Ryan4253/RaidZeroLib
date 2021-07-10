#include "Pose2D.hpp"
namespace lib4253{

Pose2D::Pose2D(const Translation2D& iTranslation, const Rotation2D& iRotation){
    translation = iTranslation;
    rotation = iRotation;
}

Pose2D::Pose2D(const okapi::QLength& x, const okapi::QLength& y, const Rotation2D& iRotation)
    : translation(x, y), rotation(iRotation){}

const Translation2D& Pose2D::getTranslation() const{
    return translation;
}

const Rotation2D& Pose2D::getRotation() const{
    return rotation;
}

okapi::QLength Pose2D::getX() const{
    return translation.getX();
}

okapi::QLength Pose2D::getY() const{
    return translation.getY();
}

okapi::QAngle Pose2D::getTheta() const{
    return rotation.getVal();
}

Pose2D Pose2D::operator+(const Transform2D& rhs) const{
    return transformBy(rhs);

}

Transform2D Pose2D::operator-(const Pose2D& rhs) const{
    const Pose2D pose = this->relativeTo(rhs);
    return Transform2D(pose.getTranslation(), pose.getRotation());
}

bool Pose2D::operator==(const Pose2D& rhs) const{
    return translation == rhs.translation && rotation == rhs.rotation;
}

bool Pose2D::operator!=(const Pose2D& rhs) const{
    return !operator==(rhs);
}

Point2D Pose2D::closestTo(const Point2D& other) const{
    /*
    Point2D current = translation;
    Point2D heading(sin(rotation.getVal()), cos(rotation.getVal()));
    Point2D n = heading.normalize();
    Point2D v = target-translation;
    double d = n*v  ;
    return (current)+((n*d));
    */
    Translation2D diff = other - translation;
    okapi::QLength mag = diff.magnitude();
    okapi::QAngle angle = (*this).angleTo(diff);
    okapi::QLength inc = mag * cos(angle);
    return translation + Translation2D(inc * cos(angle), inc * sin(angle));
}

Pose2D Pose2D::transformBy(const Transform2D& other) const{
    return {translation + (other.getTranslation().rotateBy(rotation)), rotation + other.getRotation()};
}

Pose2D Pose2D::relativeTo(const Pose2D& other) const{
    const Transform2D transform{other, *this};
    return {transform.getTranslation(), transform.getRotation()};
}

okapi::QAngle Pose2D::angleTo(const Point2D& other) const{
    return rotation.getVal() - (other-translation).angle();
}

Pose2D Pose2D::exp(const Twist2D& other) const{
    const okapi::QLength dx = other.dx;
    const okapi::QLength dy = other.dy;
    const double dtheta = other.dtheta.convert(okapi::radian);

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

    const Transform2D transform{Translation2D{dx * s - dy * c, dx * c + dy * s}, Rotation2D{cosTheta * okapi::meter, sinTheta * okapi::meter}};

    return *this + transform;
}

Twist2D Pose2D::log(const Pose2D& other) const{
    const Pose2D transform = other.relativeTo(*this);
    const double dtheta = transform.getRotation().getVal().convert(okapi::radian);
    const double halfDtheta = dtheta / 2.0;

    const double cosMinusOne = transform.getRotation().getCos() - 1;

    double halfThetaByTanOfHalfDtheta;

    if (std::abs(cosMinusOne) < 1E-9) {
      halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } 
    else {
        halfThetaByTanOfHalfDtheta =
        -(halfDtheta * transform.getRotation().getSin()) / cosMinusOne;
    }

    const Translation2D translationPart =
    transform.getTranslation().rotateBy({halfThetaByTanOfHalfDtheta, -halfDtheta}) * std::hypot(halfThetaByTanOfHalfDtheta, halfDtheta);

    return {translationPart.getX(), translationPart.getY(), (dtheta * okapi::radian)};
}

}