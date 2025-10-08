#pragma once
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include "RaidZeroLib/api/Geometry/Rotation.hpp"

namespace okapi {
class OdomState;
} // namespace okapi

namespace rz {

class Transform;
class Twist;

class Pose {
    public:
    constexpr Pose() noexcept = default;

    Pose(const Point& point, const Rotation& rotation) noexcept;

    Pose(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y, const Rotation& rotation) noexcept;

    Pose(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y, au::QuantityD<au::Radians> angle) noexcept;

    Pose(const okapi::OdomState& state) noexcept;

    const Point& getPoint() const noexcept;

    const Rotation& getRotation() const noexcept;

    au::QuantityD<au::Meters> X() const noexcept;

    au::QuantityD<au::Meters> Y() const noexcept;

    au::QuantityD<au::Radians> Theta() const noexcept;

    Pose transformBy(const Transform& rhs) const noexcept;

    Transform relativeTo(const Pose& rhs) const noexcept;

    Pose exp(const Twist& rhs) const noexcept;

    Twist log(const Pose& rhs) const noexcept;

    bool isApprox(const Pose& rhs) const noexcept;

    private:
    Point point;
    Rotation rotation;
};

au::QuantityD<au::Inverse<au::Meters>> curvatureToPoint(const Pose& pose, const Point& point) noexcept;

} // namespace rz
