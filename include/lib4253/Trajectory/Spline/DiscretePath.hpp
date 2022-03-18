#pragma once
#include "lib4253/Trajectory/Geometry/Point.hpp"
#include "lib4253/Utility/Units.hpp"
#include "lib4253/Utility/Math.hpp"
#include<vector>
#include<iostream>
#include<cmath>
namespace lib4253{
using namespace okapi;

class DiscretePath{
    public:
    DiscretePath() = default;
    DiscretePath(const std::initializer_list<Point>& iWaypoint);
    DiscretePath(const std::vector<Point>& iWaypoint);
    ~DiscretePath() = default;

    DiscretePath operator+(const DiscretePath& rhs) const;
    DiscretePath operator+(const Point& rhs) const;
    DiscretePath& operator+=(const DiscretePath& rhs);
    DiscretePath& operator+=(const Point& rhs);

    Point getPoint(int index) const;
    QCurvature getCurvature(int index) const;
    Point operator[](int index) const;
    int size() const;

    private:
    std::vector<Point> path;
};
}