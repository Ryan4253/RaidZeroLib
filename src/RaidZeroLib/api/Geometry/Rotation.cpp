#include "RaidZeroLib/api/Geometry/Rotation.hpp"
#include "RaidZeroLib/api/Utility/Math.hpp"

namespace rz {

Rotation::Rotation(au::QuantityD<au::Radians> theta) noexcept : theta(constrainAngle180(theta)) {}

Rotation::Rotation(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y) noexcept 
    : theta(x == au::ZERO && y == au::ZERO ? au::ZERO : au::arctan2(y, x)){}

Rotation::Rotation(double x, double y) noexcept
    : theta(x == 0 && y == 0 ? au::ZERO : au::radians(std::atan2(y, x))){}

au::QuantityD<au::Radians> Rotation::Theta() const noexcept {
    return theta;
}

double Rotation::Sin() const noexcept {
    return au::sin(theta);
}

double Rotation::Cos() const noexcept {
    return au::cos(theta);
}

double Rotation::Tan() const noexcept {
    return au::tan(theta);
}

Rotation Rotation::operator+(const Rotation& rhs) const noexcept {
    return Rotation(theta + rhs.theta);
}

Rotation Rotation::operator-(const Rotation& rhs) const noexcept {
    return *this + -rhs;
}

Rotation Rotation::operator-() const noexcept {
    return *this * -1;
}

Rotation Rotation::operator*(double scalar) const noexcept {
    return Rotation(theta * scalar);
}

Rotation Rotation::operator/(double scalar) const noexcept {
    return *this * (1.0 / scalar);
}

bool Rotation::isApprox(const Rotation& rhs) const noexcept {
    const auto diff = constrainAngle180(theta - rhs.theta);
    return au::abs(diff) <= au::radians(1e-9);
}

} // namespace rz