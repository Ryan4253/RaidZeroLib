#pragma once
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include "RaidZeroLib/api/Geometry/Rotation.hpp"

namespace rz {

class Pose;

class Transform {
    public:
    constexpr Transform() noexcept = default;

    Transform(const Pose& initial, const Pose& final) noexcept;

    Transform(const Point& point, const Rotation& rotation) noexcept;

    const Point& getPoint() const noexcept;

    const Rotation& getRotation() const noexcept;

    au::QuantityD<au::Meters> X() const noexcept;

    au::QuantityD<au::Meters> Y() const noexcept;

    au::QuantityD<au::Radians> Theta() const noexcept;

    Transform operator+(const Transform& rhs) const noexcept;

    Transform inverse() const noexcept;

    bool isApprox(const Transform& rhs) const noexcept;

    private:
    Point point;
    Rotation rotation;
};

} // namespace rz