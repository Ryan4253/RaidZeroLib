#pragma once
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include "RaidZeroLib/api/Geometry/Pose.hpp"

namespace rz {
class Pose;
using namespace okapi;

class Transform {
    public:
    constexpr Transform() = default;

    Transform(const Pose& iInitial, const Pose& iFinal);

    Transform(const Translation& iTranslation, const Rotation& iRotation);

    ~Transform() = default;

    const Translation& getTranslation() const;

    const Rotation& getRotation() const;

    QLength X() const;

    QLength Y() const;

    QAngle Theta() const;

    Transform operator+(const Transform& rhs) const;

    Transform operator*(double scalar) const;

    Transform operator/(double scalar) const;

    bool operator==(const Transform& rhs) const;

    bool operator!=(const Transform& rhs) const;

    void operator=(const Transform& rhs);

    Transform inverse() const;

    private:
    Translation translation;
    Rotation rotation;
};

} // namespace rz