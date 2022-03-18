/**
 * @file Point.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief  Point structs
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "Rotation.hpp"
#include "okapi/api/units/QArea.hpp"

namespace lib4253{
using namespace okapi;

class Translation{
    public:
    constexpr Translation() = default;

    Translation(QLength iX, QLength iY);

    Translation(QLength iMag, const Rotation& iAngle);

    Translation(const Translation& rhs);

    ~Translation() = default;

    QLength X() const;

    QLength Y() const;

    Translation operator+(const Translation& rhs) const;

    Translation operator-(const Translation& rhs) const;

    Translation operator-() const;

    Translation operator*(double scalar) const;

    QArea operator*(const Translation& rhs) const;

    Translation operator/(double scalar) const;

    bool operator==(const Translation& rhs) const;

    bool operator!=(const Translation& rhs) const;

    void operator=(const Translation& rhs);

    QAngle Theta() const;

    QLength distTo(const Translation& rhs) const;

    QAngle angleTo(const Translation& rhs) const;

    QLength mag() const;

    Translation rotateBy(const Rotation& rhs) const;

    void setX(QLength iX);

    void setY(QLength iY);
    
    private:
    QLength x = 0 * meter;
    QLength y = 0 * meter;
};

typedef Translation Point;
}

