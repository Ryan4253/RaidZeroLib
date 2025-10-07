#pragma once
#include "au/au.hpp"

namespace rz {

class Rotation {
    public:
    constexpr Rotation() noexcept = default;

    explicit Rotation(au::QuantityD<au::Radians> theta) noexcept;

    Rotation(au::QuantityD<au::Meters> x, au::QuantityD<au::Meters> y) noexcept;

    au::QuantityD<au::Radians> Theta() const noexcept;

    double Sin() const noexcept;

    double Cos() const noexcept;

    double Tan() const noexcept;

    Rotation operator+(const Rotation& rhs) const noexcept;

    Rotation operator-(const Rotation& rhs) const noexcept;

    Rotation operator-() const noexcept;

    Rotation operator*(double scalar) const noexcept;

    Rotation operator/(double scalar) const noexcept;

    bool isApprox(const Rotation& rhs, au::QuantityD<au::Radians> tol = au::radians(1e-12)) const noexcept;

    private:
    au::QuantityD<au::Radians> theta = au::ZERO;
};

} // namespace rz
