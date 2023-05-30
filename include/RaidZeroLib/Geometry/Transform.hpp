#pragma once
#include "Point.hpp"
#include "Pose.hpp"

namespace lib4253{
class Pose;
using namespace okapi;

class Transform{
    public:
    constexpr Transform() = default;

    Transform(const Pose& initial, const Pose& final);

    Transform(const Translation& iTranslation, const Rotation& iRotation);

    ~Transform() = default;

    const Translation& getTranslation() const;

    const Rotation& getRotation() const;

    QLength X() const;

    QLength Y() const;

    bool operator==(const Transform& rhs) const;

    bool operator!=(const Transform& rhs) const;

    void operator=(const Transform& rhs);

    Transform operator*(double scalar) const;

    Transform operator/(double scalar) const;

    Transform inverse() const;

    private:
    Translation translation;
    Rotation rotation;
};

}