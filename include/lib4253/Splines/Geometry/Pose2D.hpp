#pragma once
#include "Point2D.hpp"
#include "Transform2D.hpp"
#include "Twist2D.hpp"

namespace lib4253{
class Pose2D{
    public:
    constexpr Pose2D() = default;

    Pose2D(const Translation2D& iTranslation, const Rotation2D& iRotation);

    Pose2D(const okapi::QLength& x, const okapi::QLength& y, const Rotation2D& iRotation);

    ~Pose2D() = default;

    const Translation2D& getTranslation() const;

    const Rotation2D& getRotation() const;

    okapi::QLength getX() const;

    okapi::QLength getY() const;

    okapi::QAngle getTheta() const;

    Pose2D operator+(const Transform2D& rhs) const;
    
    Transform2D operator-(const Pose2D& rhs) const;

    bool operator==(const Pose2D& rhs) const;

    bool operator!=(const Pose2D& rhs) const;

    Pose2D transformBy(const Transform2D& other) const;

    Pose2D relativeTo(const Pose2D& other) const;

    Pose2D exp(const Twist2D& other) const;

    Twist2D log(const Pose2D& other) const;

    Translation2D translation;
    Rotation2D rotation;
};
}

