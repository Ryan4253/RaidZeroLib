#pragma once
#include "lib4253/Trajectory/Geometry/Point2D.hpp"
#include "lib4253/Trajectory/Spline/DiscretePath.hpp"
#include "lib4253/Utility/Math.hpp"
#include <stack>
#include<vector>
namespace lib4253{

class Bezier{
    public:
    Bezier(std::vector<Point2D> control_point);

    Point2D getFirstPoint();

    Point2D getLastPoint();

    Point2D at(double t);

    DiscretePath generate(int iStep, bool end);

    private:
    std::vector<Point2D> waypoint;
    okapi::QLength maxdX;
    okapi::QLength maxdY;
    okapi::QAngle maxdTheta;

};
}
