#include "RaidZeroLib/api/Geometry/Pose.hpp"

namespace rz {

Pose::Pose(const Translation& iTranslation, const Rotation& iRotation)
    : translation(iTranslation), rotation(iRotation) {
}

Pose::Pose(QLength iX, QLength iY, const Rotation& iRotation) : translation(iX, iY), rotation(iRotation) {
}

Pose::Pose(const OdomState& iState) {
    translation = Translation(iState.x, -1 * iState.y);
    rotation = Rotation(iState.theta);
}

const Translation& Pose::getTranslation() const {
    return translation;
}

const Rotation& Pose::getRotation() const {
    return rotation;
}

QLength Pose::X() const {
    return translation.X();
}

QLength Pose::Y() const {
    return translation.Y();
}

QAngle Pose::Theta() const {
    return rotation.Theta();
}

Pose Pose::operator+(const Transform& rhs) const {
    return transformBy(rhs);
}

Transform Pose::operator-(const Pose& rhs) const {
    const Pose pose = this->relativeTo(rhs);
    return Transform(pose.getTranslation(), pose.getRotation());
}

Pose Pose::operator*(double scalar) const {
    return Pose(translation * scalar, rotation * scalar);
}

Pose Pose::operator/(double scalar) const {
    return *this * (1.0 / scalar);
}

bool Pose::operator==(const Pose& rhs) const {
    return translation == rhs.translation && rotation == rhs.rotation;
}

bool Pose::operator!=(const Pose& rhs) const {
    return !operator==(rhs);
}

void Pose::operator=(const Pose& rhs) {
    translation = rhs.getTranslation();
    rotation = rhs.getRotation();
}

Pose Pose::transformBy(const Transform& rhs) const {
    return Pose(translation + rhs.getTranslation().rotateBy(rotation), rotation + rhs.getRotation());
}

Pose Pose::relativeTo(const Pose& rhs) const {
    const Transform transform(rhs, *this);
    return Pose(transform.getTranslation(), transform.getRotation());
}

Pose Pose::exp(const Twist& rhs) const {
    const QLength dx = rhs.dX();
    const QLength dy = rhs.dY();
    const double dtheta = rhs.dTheta().convert(radian);

    const double sinTheta = std::sin(dtheta);
    const double cosTheta = std::cos(dtheta);

    double s, c;
    if (std::abs(dtheta) < 1E-9) {
        s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
        c = 0.5 * dtheta;
    } else {
        s = sinTheta / dtheta;
        c = (1 - cosTheta) / dtheta;
    }

    const Transform transform(Translation{dx * s - dy * c, dx * c + dy * s},
                              Rotation{cosTheta * meter, sinTheta * meter});

    return *this + transform;
}

Twist Pose::log(const Pose& rhs) const {
    const Pose transform = rhs.relativeTo(*this);
    const double dtheta = transform.getRotation().Theta().convert(radian);
    const double halfDtheta = dtheta / 2.0;

    const double cosMinusOne = transform.getRotation().Cos() - 1;

    double halfThetaByTanOfHalfDtheta;

    if (std::abs(cosMinusOne) < 1E-9) {
        halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
        halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.getRotation().Sin()) / cosMinusOne;
    }

    const Translation translationPart =
        transform.getTranslation().rotateBy(Rotation(halfThetaByTanOfHalfDtheta, -halfDtheta)) *
        std::hypot(halfThetaByTanOfHalfDtheta, halfDtheta);

    return {translationPart.X(), translationPart.Y(), (dtheta * radian)};
}

QCurvature curvatureToReachPoint(const Pose& position, const Point& point) {
    const double a = -tan(position.Theta()).convert(number);
    const double b = 1;
    const QLength c = tan(position.Theta()) * position.X() - position.Y();

    const QLength x = abs(point.X() * a + point.Y() * b + c) / sqrt(a * a + b * b);
    const QLength sideL =
        sin(position.Theta()) * (point.X() - position.X()) - cos(position.Theta()) * (point.Y() - position.Y());
    const Number side = sideL / abs(sideL);

    if (sideL.convert(meter) == 0) {
        return 0 * radpm;
    }

    const QLength chord = position.getTranslation().distTo(point);

    return (2 * x) / (chord * chord) * radian * side;
}

} // namespace rz