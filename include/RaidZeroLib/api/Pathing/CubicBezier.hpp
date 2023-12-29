#pragma once
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include "RaidZeroLib/api/Pathing/ParametricPath.hpp"
#include "RaidZeroLib/api/Units/Units.hpp"

namespace rz {
using namespace okapi;

class CubicBezier : public ParametricPath {
    public:
    class Knot {
        public:
        Knot(QLength x, QLength y, QAngle theta, QLength magnitude);

        Point getPoint() const;

        Point getForwardControl() const;

        Point getBackwardControl() const;

        private:
        QLength x;
        QLength y;
        QAngle theta;
        QLength magnitude;
    };

    CubicBezier(Knot start, Knot end);

    Point getPoint(double t) const override;

    Point getVelocity(double t) const override;

    Point getAcceleration(double t) const override;

    private:
    Point c0;
    Point c1;
    Point c2;
    Point c3;
};

} // namespace rz