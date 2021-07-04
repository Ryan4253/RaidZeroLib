#pragma once
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace lib4253{
class Rotation2D{
    public:
    constexpr Rotation2D() = default;

    Rotation2D(const okapi::QAngle& val);

    Rotation2D(const okapi::QLength& x, const okapi::QLength& y);

    Rotation2D(const double& x, const double& y);

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

    Rotation2D rotateBy(const Rotation2D& other) const;

    okapi::QAngle value = 0 * okapi::radian;
    double cosine = 1;
    double sine = 0;
};

}