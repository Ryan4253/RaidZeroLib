#include "RaidZeroLib/api/Geometry/Transform.hpp"
#include "RaidZeroLib/api/Geometry/Pose.hpp"
#include "au/au.hpp"

namespace rz {

Transform::Transform(const Pose& initial, const Pose& final) noexcept 
    : point((final.getPoint() - initial.getPoint()).rotateBy(-initial.getRotation())), 
      rotation(final.getRotation() - initial.getRotation()){}

Transform::Transform(const Point& point, const Rotation& rotation) noexcept
    : point(point), rotation(rotation) {}

const Point& Transform::getPoint() const noexcept {
    return point;
}

const Rotation& Transform::getRotation() const noexcept {
    return rotation;
}

au::QuantityD<au::Meters> Transform::X() const noexcept {
    return point.X();
}

au::QuantityD<au::Meters> Transform::Y() const noexcept {
    return point.Y();
}

au::QuantityD<au::Radians> Transform::Theta() const noexcept {
    return rotation.Theta();
}

Transform Transform::operator+(const Transform& rhs) const noexcept {
    return Transform(Pose(), Pose().transformBy(*this).transformBy(rhs));
}

Transform Transform::operator-() const noexcept {
    return Transform((-point).rotateBy(-rotation), -rotation);
}

bool Transform::isApprox(const Transform& rhs) const noexcept {
    return point.isApprox(rhs.point) && rotation.isApprox(rhs.rotation);
}

} // namespace rz
