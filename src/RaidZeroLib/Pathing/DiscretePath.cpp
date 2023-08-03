#include "RaidZeroLib/Pathing/DiscretePath.hpp"

namespace rz{
DiscretePath::DiscretePath(const std::initializer_list<Point>& waypoint) : path(waypoint){}

DiscretePath::DiscretePath(const std::vector<Point>& waypoint) : path(waypoint){}

DiscretePath DiscretePath::operator+(const DiscretePath& rhs) const{
    DiscretePath result(path);
    result.path.insert(result.path.end(), rhs.path.begin(), rhs.path.end());
    return result;
}

DiscretePath DiscretePath::operator+(const Point& rhs) const{
    DiscretePath result(path);
    result.path.emplace_back(rhs);
    return result;
}

DiscretePath& DiscretePath::operator+=(const DiscretePath& rhs){
    path.insert(path.end(), rhs.path.begin(), rhs.path.end());
    return *this;
}

DiscretePath& DiscretePath::operator+=(const Point& rhs){
    path.emplace_back(rhs);
    return *this;
}

std::vector<Point>::iterator DiscretePath::begin(){
    return path.begin();
}

std::vector<Point>::iterator DiscretePath::end(){
    return path.end();
}

std::vector<Point>::reverse_iterator DiscretePath::rbegin(){
    return path.rbegin();
}

std::vector<Point>::reverse_iterator DiscretePath::rend(){
    return path.rend();
}

std::vector<Point>::const_iterator DiscretePath::begin() const{
    return path.begin();
}

std::vector<Point>::const_iterator DiscretePath::end() const{
    return path.end();
}

std::vector<Point>::const_reverse_iterator DiscretePath::rbegin() const{
    return path.rbegin();
}

std::vector<Point>::const_reverse_iterator DiscretePath::rend() const{
    return path.rend();
}

Point& DiscretePath::operator[](int index){
    return path[index];
}

const Point& DiscretePath::operator[](int index) const{
    return path[index];
}

Point& DiscretePath::front(){
    return path.front();
}

const Point& DiscretePath::front() const{
    return path.front();
}

Point& DiscretePath::back(){
    return path.back();
}

const Point& DiscretePath::back() const{
    return path.back();
}

int DiscretePath::size() const{
    return (int)path.size();
}

QCurvature DiscretePath::getCurvature(int index) const{
    if(index <= 0 || index >= (int)path.size()-1){
        return 0 * radpm;
    }

    QLength radius = circumradius(path[index-1], path[index], path[index+1]);

    if(std::isnan(radius.getValue())){
        return 0 * radpm;
    }

    return 1 * radian / radius;
}

std::vector<Translation>::iterator closestPoint(std::vector<Translation>::iterator begin, std::vector<Translation>::iterator end, const Point& point){
    const auto comparison = [point](const Point& a, const Point& b){
        return a.distTo(point) < b.distTo(point);
    };

    return std::min_element(begin, end, comparison);
}



} 