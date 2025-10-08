#include "RaidZeroLib/api/Geometry/Pose.hpp"
#include "RaidZeroLib/api/Geometry/Transform.hpp"
#include "RaidZeroLib/api/Geometry/Twist.hpp"
#include "okapi/api/odometry/odomState.hpp"
#include <functional>

namespace rz {

Pose::Pose(const Point& point, const Rotation& rotation) noexcept
    : point(point), rotation(rotation) {}

Pose::Pose(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y, const Rotation& rotation) noexcept
    : point(x, y), rotation(rotation) {}

Pose::Pose(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y, au::QuantityD<au::Radians> angle) noexcept 
    : point(x, y), rotation(angle){}

Pose::Pose(const okapi::OdomState& state) noexcept {
    const double x = state.x.convert(okapi::meter);
    const double y = -state.y.convert(okapi::meter);
    const double theta = -state.theta.convert(okapi::radian);

    point = Point(au::meters(x), au::meters(y));
    rotation = Rotation(au::radians(theta));
}

const Point& Pose::getPoint() const noexcept {
    return point;
}

const Rotation& Pose::getRotation() const noexcept {
    return rotation;
}

au::QuantityD<au::Meters> Pose::X() const noexcept {
    return point.X();
}

au::QuantityD<au::Meters> Pose::Y() const noexcept {
    return point.Y();
}

au::QuantityD<au::Radians> Pose::Theta() const noexcept {
    return rotation.Theta();
}

Pose Pose::transformBy(const Transform& rhs) const noexcept {
    return Pose(point + rhs.getPoint().rotateBy(rotation), rotation + rhs.getRotation());
}

Transform Pose::relativeTo(const Pose& rhs) const noexcept {
    return Transform(rhs, *this);
}

Pose Pose::exp(const Twist& rhs) const noexcept {
    const auto dx = rhs.dX();
    const auto dy = rhs.dY();
    const auto dtheta = rhs.dTheta();

    const double sinTheta = au::sin(dtheta);
    const double cosTheta = au::cos(dtheta);

    double s, c;
    if (au::abs(dtheta) < au::radians(1e-9)) {
        s = 1.0 - 1.0 / 6.0 * dtheta.in(au::radian) * dtheta.in(au::radian);
        c = 0.5 * dtheta.in(au::radian);
    } else {
        s = sinTheta / dtheta.in(au::radian);
        c = (1 - cosTheta) / dtheta.in(au::radian);
    }

    const Transform transform(Point{dx * s - dy * c, dx * c + dy * s}, Rotation{cosTheta, sinTheta});

    return this->transformBy(transform);
}

Twist Pose::log(const Pose& rhs) const noexcept {
    const Transform transform = rhs.relativeTo(*this);
    const auto dtheta = transform.getRotation().Theta();
    const auto halfDtheta = dtheta / 2;

    const double a = std::invoke([&dtheta, &halfDtheta](){
        if(au::abs(dtheta) < au::radians(1e-9)){
            return 1.0 - (dtheta.in(au::radians) * dtheta.in(au::radians)) / 12.0; 
        }
        else{
            return halfDtheta.in(au::radian) / au::tan(halfDtheta);
        }
    });

    const double b = halfDtheta.in(au::radians);

    const auto dx = a * transform.X() + b * transform.Y();
    const auto dy = -b * transform.X() + a * transform.Y();

    return Twist(dx, dy, dtheta);
}

bool Pose::isApprox(const Pose& rhs) const noexcept {
    return point.isApprox(rhs.point) && rotation.isApprox(rhs.rotation);
}

au::QuantityD<au::Inverse<au::Meters>> curvatureToPoint(const Pose& pose, const Point& point) noexcept {
    const double a = -au::tan(pose.Theta());
    const double b = 1;
    const auto c = au::tan(pose.Theta()) * pose.X() - pose.Y();

    const auto x = au::abs(point.X() * a + point.Y() * b + c) / au::sqrt(a * a + b * b);
    const au::QuantityD<au::Meters> sideL =
        au::sin(pose.Theta()) * (point.X() - pose.X()) - cos(pose.Theta()) * (point.Y() - pose.Y());

    if(sideL == au::ZERO){
        return au::ZERO;
    }

    const double side = sideL / au::abs(sideL);
    const au::QuantityD<au::Meters> chord = pose.getPoint().distTo(point);

    return (2 * x) / (chord * chord) * side;
}

} // namespace rz
