#pragma once
#include "RaidZeroLib/Geometry/Point.hpp"
#include<vector>   
#include<exception>
#include<algorithm>

namespace rz{
using namespace okapi;

class DiscretePath{
    public:
    DiscretePath() = default;
    DiscretePath(const std::initializer_list<Point>& waypoint);
    DiscretePath(const std::vector<Point>& waypoint);
    ~DiscretePath() = default;

    DiscretePath operator+(const DiscretePath& rhs) const;
    DiscretePath operator+(const Point& rhs) const;
    DiscretePath& operator+=(const DiscretePath& rhs);
    DiscretePath& operator+=(const Point& rhs);

    std::vector<Point>::iterator begin();
    std::vector<Point>::iterator end();
    std::vector<Point>::reverse_iterator rbegin();
    std::vector<Point>::reverse_iterator rend();

    Point& operator[](int index);
    Point& front();
    Point& back();
    Point& getPoint(int index);
    QCurvature getCurvature(int index) const;
    int size() const;

    private:
    std::vector<Point> path;
};

template<typename Iterator>
Iterator closestPoint(Iterator begin, Iterator end, const Point& point);

}