#pragma once
#include "lib4253/Trajectory/Spline/Bezier.hpp"
namespace lib4253{

class CompoundBezier{
    public:
    CompoundBezier(std::vector<Bezier> curve);

    Point2D at(double t);

    DiscretePath generate(int iStep, bool end);

    private:
    std::vector<Bezier> path;
};

}