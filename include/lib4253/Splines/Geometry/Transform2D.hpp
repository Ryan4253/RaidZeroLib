#pragma once
#include "lib4253/Splines/Geometry/Point2D.hpp"

namespace lib4253{
class Pose2D;

class Transform2D{
    public:
    constexpr Transform2D() = default;

    Transform2D(const Pose2D& initial, const Pose2D& final);

    Transform2D(const Translation2D& iTranslation, const Rotation2D& iRotation);

    ~Transform2D() = default;

    const Translation2D& getTranslation() const;

    const Rotation2D& getRotation() const;

    okapi::QLength getX() const;

    okapi::QLength getY() const;

    bool operator==(const Transform2D& rhs) const;

    bool operator!=(const Transform2D& rhs) const;

    Transform2D operator*(double scalar) const;

    Transform2D inverse() const;

    Translation2D translation;
    Rotation2D rotation;
};

}