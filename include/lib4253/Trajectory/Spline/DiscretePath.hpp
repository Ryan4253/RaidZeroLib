#pragma once
#include "lib4253/Trajectory/Geometry/Point2D.hpp"
#include "lib4253/Utility/Units.hpp"
#include<vector>
#include<iostream>
#include<cmath>
namespace lib4253{

class DiscretePath{
    public:
    DiscretePath() = default;
    DiscretePath(const std::initializer_list<Point2D>& waypoint);
    DiscretePath(const std::vector<Point2D>& waypoint);
    ~DiscretePath() = default;

    DiscretePath operator+(const DiscretePath& rhs) const;
    DiscretePath operator+(const Point2D& rhs) const;
    DiscretePath& operator+=(const DiscretePath& rhs);
    DiscretePath& operator+=(const Point2D& rhs);
    Point2D operator[](const int& index) const;

    DiscretePath& generate(int step, bool end = true);
    DiscretePath& generate(okapi::QLength dist, bool end = true);
    DiscretePath& smooth(double a, double b, okapi::QLength tolerance);

    int getSize() const;
    okapi::QCurvature getCurvature(const int& index) const;

    private:
    std::vector<Point2D> path;
};
}