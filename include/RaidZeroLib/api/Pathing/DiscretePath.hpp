#pragma once
#include "RaidZeroLib/api/Geometry/Point.hpp"
#include <algorithm>
#include <exception>
#include <vector>

namespace rz {
using namespace okapi;

class DiscretePath {
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
    std::vector<Point>::const_iterator begin() const;
    std::vector<Point>::iterator end();
    std::vector<Point>::const_iterator end() const;

    std::vector<Point>::reverse_iterator rbegin();
    std::vector<Point>::const_reverse_iterator rbegin() const;
    std::vector<Point>::reverse_iterator rend();
    std::vector<Point>::const_reverse_iterator rend() const;

    Point& operator[](int index);
    const Point& operator[](int index) const;
    Point& front();
    const Point& front() const;
    Point& back();
    const Point& back() const;
    QCurvature getCurvature(int index) const;
    int size() const;

    private:
    std::vector<Point> path;
};

std::vector<Translation>::iterator closestPoint(std::vector<Translation>::iterator begin,
                                                std::vector<Translation>::iterator end, const Point& point);
} // namespace rz