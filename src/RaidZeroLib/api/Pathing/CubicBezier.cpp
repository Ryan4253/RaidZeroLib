#include "RaidZeroLib/api/Pathing/CubicBezier.hpp"
#include "RaidZeroLib/api/Geometry/Rotation.hpp"
#include <cassert>

namespace rz {

CubicBezier::Knot::Knot(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y, 
                                        au::QuantityD<au::Radians> theta, au::QuantityD<au::Meters> magnitude) noexcept
    : x(x), y(y), theta(theta), magnitude(magnitude){
    assert(magnitude > au::ZERO);
};

Point CubicBezier::Knot::getPoint() const noexcept {
    return Point(x, y);
}

Point CubicBezier::Knot::getForwardControl() const noexcept {
    return getPoint() + Point(magnitude, Rotation(theta));
}

Point CubicBezier::Knot::getBackwardControl() const noexcept {
    return getPoint() - Point(magnitude, Rotation(theta));
}

CubicBezier::CubicBezier(const Knot& start, const Knot& end) noexcept
    : c0(start.getPoint()), c1(start.getForwardControl()), c2(end.getBackwardControl()), c3(end.getPoint()){};

Point CubicBezier::getPoint(double t) const noexcept {
    // clang-format off
    return c0 * (1 - t) * (1 - t) * (1 - t) + 
           c1 * 3 * (1 - t) * (1 - t) * t +
           c2 * 3 * (1 - t) * t * t + 
           c3 * t * t * t;
    // clang-format on
}

Point CubicBezier::getVelocity(double t) const noexcept {
    // clang-format off
    return (c1 - c0) * 3 * (1 - t) * (1 - t) + 
           (c2 - c1) * 6 * (1 - t) * t  + 
           (c3 - c2) * 3 * t * t;
    // clang-format on
}

Point CubicBezier::getAcceleration(double t) const noexcept {
    // clang-format off
    return (c2 - c1 * 2 + c0) * 6 * (1 - t) + 
           (c3 - c2 * 2 + c1) * 6 * t;
    // clang-format on
}

} // namespace rz
