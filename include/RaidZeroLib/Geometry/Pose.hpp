#pragma once
#include "RaidZeroLib/Geometry/Point.hpp"
#include "RaidZeroLib/Geometry/Transform.hpp"
#include "RaidZeroLib/Geometry/Twist.hpp"
#include "okapi/api/odometry/odomState.hpp"

namespace rz{
using namespace okapi;
class Transform;

class Pose{
    public:
    constexpr Pose() = default;

    Pose(const Translation& iTranslation, const Rotation& iRotation);

    Pose(QLength iX, QLength iY, const Rotation& iRotation);

    Pose(const OdomState& iState);

    ~Pose() = default;

    const Translation& getTranslation() const;

    const Rotation& getRotation() const;

    QLength X() const;

    QLength Y() const;

    QAngle Theta() const;

    Pose operator+(const Transform& rhs) const;
    
    Transform operator-(const Pose& rhs) const;

    Pose operator*(double scalar) const;

    Pose operator/(double scalar) const;

    bool operator==(const Pose& rhs) const;

    bool operator!=(const Pose& rhs) const;

    void operator=(const Pose& rhs);

    Pose transformBy(const Transform& rhs) const;

    Pose relativeTo(const Pose& rhs) const;

    Pose exp(const Twist& rhs) const;

    Twist log(const Pose& rhs) const;

    private:
    Translation translation;
    Rotation rotation;
};

}

