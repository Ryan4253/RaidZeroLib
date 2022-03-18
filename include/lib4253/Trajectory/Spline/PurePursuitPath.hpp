#pragma once
#include "lib4253/Trajectory/Spline/DiscretePath.hpp"
#include "lib4253/Utility/Units.hpp"
namespace lib4253{
using namespace okapi;

struct PurePursuitGains{
    QSpeed maxVelocity;
    QAcceleration maxAcceleration;
    QAngularSpeed maxAngularVelocity;
    PurePursuitGains(QSpeed iMaxVelocity, QAcceleration iMaxAcceleration, QAngularSpeed iMaxAngularVelocity);
};

class PurePursuitPath{
    public:
    PurePursuitPath() = default;
    PurePursuitPath(const DiscretePath& iPath, const PurePursuitGains& iLimits);

    int size() const;
    Point getPoint(int index) const;
    QSpeed getVelocity(int index) const;
    QAcceleration getAcceleration(int index) const;
    QCurvature getCurvature(int index) const;
    Point operator[](int index) const;

    private:
    DiscretePath path;
    std::vector<QSpeed> velocity;
    std::vector<QAcceleration> acceleration;
};

}