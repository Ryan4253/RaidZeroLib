#pragma once
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "lib4253/Utility/Units.hpp"

namespace lib4253{
class Rotation2D{
    public:
    constexpr Rotation2D() = default;

    Rotation2D(okapi::QAngle val);

    Rotation2D(okapi::QLength x, okapi::QLength y);

    ~Rotation2D() = default;

    okapi::QAngle getVal() const;

    double getSin() const;

    double getCos() const;

    double getTan() const;

    Rotation2D operator+(const Rotation2D& rhs) const;

    Rotation2D operator-(const Rotation2D& rhs) const;

    Rotation2D operator-() const;

    Rotation2D operator*(const double& scalar) const;

    bool operator==(const Rotation2D& rhs) const;

    bool operator!=(const Rotation2D& rhs) const;

    Rotation2D rotateBy(const Rotation2D& rhs) const;

    private:
    okapi::QAngle value = 0 * okapi::radian;
    double cosine = 1;
    double sine = 0;
};

}