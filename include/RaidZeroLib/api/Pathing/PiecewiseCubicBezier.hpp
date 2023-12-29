#pragma once
#include "RaidZeroLib/api/Pathing/CubicBezier.hpp"

namespace rz {
using namespace okapi;

class PiecewiseCubicBezier : public ParametricPath {
    public:
    PiecewiseCubicBezier(std::initializer_list<CubicBezier::Knot> knots);

    Point getPoint(double t) const override;

    Point getVelocity(double t) const override;

    Point getAcceleration(double t) const override;

    private:
    std::pair<int, double> getIndex(double t) const;

    std::vector<CubicBezier> curves;
};

} // namespace rz