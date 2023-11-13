#pragma once
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace rz {
using namespace okapi;

class Twist {
    public:
    Twist(QLength dX, QLength dY, QAngle dTheta);

    Twist(const Twist& rhs);

    ~Twist() = default;

    QLength dX() const;

    QLength dY() const;

    QAngle dTheta() const;

    bool operator==(const Twist& rhs) const;

    bool operator!=(const Twist& rhs) const;

    void operator=(const Twist& rhs);

    private:
    QLength dx{0.0};
    QLength dy{0.0};
    QAngle dtheta{0.0};
};

} // namespace rz