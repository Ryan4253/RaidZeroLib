#pragma once
#include "lib4253/Splines/Geometry/Point2D.hpp"

namespace lib4253{
class Pose2D;

class Transform2D{
    public:
    Transform2D(Pose2D initial, Pose2D final);

    Transform2D(Translation2D trans, Rotation2D rot);

    constexpr Transform2D() = default;

    const Transform2D& getTranslation() const;

    const Rotation2D& getRotation() const;

    okapi::QLength getX() const;

    okapi::QLength getY() const;

    Transform2D Inverse() const;

    Transform2D operator*(double scalar) const;

    bool operator==(const Transform2D& other) const;

    bool operator!=(const Transform2D& other) const;

    private:
    Translation2D translation;
    Rotation2D rotation;
};

}