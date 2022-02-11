#pragma once
#include "Point.hpp"
#include "Transform.hpp"
#include "Twist.hpp"

namespace lib4253{
using namespace okapi;
class Transform;

class Pose{
    public:
    constexpr Pose() = default;

    Pose(const Translation& iTranslation, const Rotation& iRotation);

    Pose(QLength iX, QLength iY, const Rotation& iRotation);

    ~Pose() = default;

    const Translation& getTranslation() const;

    const Rotation& getRotation() const;

    QLength X() const;

    QLength Y() const;

    QAngle Theta() const;

    Pose operator+(const Transform& rhs) const;
    
    Transform operator-(const Pose& rhs) const;

    bool operator==(const Pose& rhs) const;

    bool operator!=(const Pose& rhs) const;

    void operator=(const Pose& rhs);

    Point closestTo(const Point& rhs) const;

    Pose transformBy(const Transform& rhs) const;

    Pose relativeTo(const Pose& rhs) const;

    QAngle angleTo(const Point& rhs) const;

    Pose exp(const Twist& rhs) const;

    Twist log(const Pose& rhs) const;

    private:
    Translation translation;
    Rotation rotation;
};
}

