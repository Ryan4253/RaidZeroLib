#pragma once
#include "RaidZeroLib/Geometry/Rotation.hpp"
#include "okapi/api/units/QArea.hpp"
#include<optional>

namespace rz{
using namespace okapi;
class Rotation;

class Translation{
    public:
    constexpr Translation() = default;

    Translation(QLength iX, QLength iY);

    Translation(QLength iMag, const Rotation& iAngle);

    Translation(const Translation& rhs);

    ~Translation() = default;

    QLength X() const;

    QLength Y() const;

    void setX(QLength iX);

    void setY(QLength iY);

    Translation operator+(const Translation& rhs) const;

    Translation operator-(const Translation& rhs) const;

    Translation operator-() const;

    Translation operator*(double scalar) const;

    Translation operator/(double scalar) const;

    bool operator==(const Translation& rhs) const;

    bool operator!=(const Translation& rhs) const;

    void operator=(const Translation& rhs);

    QAngle theta() const;

    QLength mag() const;

    QLength distTo(const Translation& rhs) const;

    QAngle angleTo(const Translation& rhs) const;

    QArea dot(const Translation& rhs) const;

    QArea wedge(const Translation& rhs) const;

    Translation project(const Translation& rhs) const;

    Translation rotateBy(const Rotation& rhs) const;

    private:
    QLength x = 0 * meter;
    QLength y = 0 * meter;
};

using Point = Translation;

QLength circumradius(const Translation& iLeft, const Translation& iMid, const Translation& iRight);

std::optional<double> circleLineIntersection(const Translation& start, const Translation& end, const Translation& point, QLength radius);

}

