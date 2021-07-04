/**
 * @file Point2D.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief 2D Point structs
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "Rotation2D.hpp"
#include "okapi/api/units/QArea.hpp"

namespace lib4253{

class Translation2D{
    public:
    constexpr Translation2D() = default;

    Translation2D(const okapi::QLength& xPos, const okapi::QLength& yPos);

    Translation2D(const okapi::QLength& magnitude, const Rotation2D& angle);

    ~Translation2D() = default;

    okapi::QLength getX() const;

    okapi::QLength getY() const;

    Translation2D operator+(const Translation2D& rhs) const;

    Translation2D operator-(const Translation2D& rhs) const;

    Translation2D operator-() const;

    Translation2D operator*(const double& scalar) const;

    okapi::QArea operator*(const Translation2D& rhs) const;

    Translation2D operator/(const double& scalar) const;

    bool operator==(const Translation2D& rhs) const;

    bool operator!=(const Translation2D& rhs) const;

    okapi::QLength distanceTo(const Translation2D& other) const;

    okapi::QAngle angleTo(const Translation2D& other) const;

    okapi::QLength magnitude() const;

    Translation2D rotateBy(const Rotation2D& other) const;

    okapi::QLength x = 0 * okapi::meter;
    okapi::QLength y = 0 * okapi::meter;

};

typedef Translation2D Point2D;
}

