#pragma once
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace lib4253{
using namespace okapi;

class Rotation{
    public:
    constexpr Rotation() = default;

    Rotation(QAngle iTheta);

    Rotation(QLength iX, QLength iY);

    Rotation(double iX, double iY);

    ~Rotation() = default;

    QAngle Theta() const;

    double Sin() const;

    double Cos() const;

    double Tan() const;

    Rotation operator+(const Rotation& rhs) const;

    Rotation operator-(const Rotation& rhs) const;

    Rotation operator-() const;

    Rotation operator*(double scalar) const;

    Rotation operator/(double scalar) const;

    bool operator==(const Rotation& rhs) const;

    bool operator!=(const Rotation& rhs) const;

    void operator=(const Rotation& rhs);

    Rotation rotateBy(const Rotation& rhs) const;

    private:
    QAngle theta{0.0};
    double cosine{1.0};
    double sine{0.0};
};

}
