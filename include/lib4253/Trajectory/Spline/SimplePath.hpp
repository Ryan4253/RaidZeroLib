#pragma once
#include "DiscretePath.hpp"

namespace lib4253{
using namespace okapi;

class SimplePath{
    public:
    SimplePath() = default;
    SimplePath(const std::initializer_list<Point>& iWaypoint);
    ~SimplePath() = default;

    SimplePath& generate(int iStep, bool iEnd = true);
    SimplePath& generate(QLength iLength, bool iEnd = true);
    DiscretePath smoothen(double iSmoothWeight, QLength iTolerance);
    DiscretePath noSmoothen();

    private:
    std::vector<Point> waypoint;
};


}
