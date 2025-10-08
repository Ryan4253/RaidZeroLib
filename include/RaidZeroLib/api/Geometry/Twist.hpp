#pragma once
#include "au/au.hpp"

namespace rz {

class Twist {
    public:
    Twist(au::QuantityD<au::Meters> dx, au::QuantityD<au::Meters> dy, au::QuantityD<au::Radians> dtheta) noexcept;

    au::QuantityD<au::Meters> dX() const noexcept;

    au::QuantityD<au::Meters> dY() const noexcept;

    au::QuantityD<au::Radians> dTheta() const noexcept;

    bool isApprox(const Twist& rhs) const noexcept;

    private:
    au::QuantityD<au::Meters> dx = au::ZERO;
    au::QuantityD<au::Meters> dy = au::ZERO;
    au::QuantityD<au::Radians> dtheta = au::ZERO;
};

} // namespace rz