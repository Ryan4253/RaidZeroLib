#include "lib4253/Trajectory/Spline/DiscretePath.hpp"
namespace lib4253{

DiscretePath::DiscretePath(const std::initializer_list<Point>& iWaypoint) : path(iWaypoint){}

DiscretePath::DiscretePath(const std::vector<Point>& iWaypoint) : path(iWaypoint){}

DiscretePath DiscretePath::operator+(const DiscretePath& rhs) const{
    DiscretePath result(path);
    result.path.insert(result.path.end(), rhs.path.begin(), rhs.path.end());
    return result;
}

DiscretePath DiscretePath::operator+(const Point& rhs) const{
    DiscretePath result(path);
    result.path.push_back(rhs);
    return result;
}

DiscretePath& DiscretePath::operator+=(const DiscretePath& rhs){
    path.insert(path.end(), rhs.path.begin(), rhs.path.end());
    return *this;
}

DiscretePath& DiscretePath::operator+=(const Point& rhs){
    path.push_back(rhs);
    return *this;
}

Point DiscretePath::getPoint(int index) const{
    if(index < 0 || index >= (int)path.size()) return Point(0 * meter, 0 * meter);
    return path[index];
}

Point DiscretePath::operator[](int index) const{
    return getPoint(index);
}


int DiscretePath::size() const{
    return (int)path.size();
}

QCurvature DiscretePath::getCurvature(int index) const{
    if(index <= 0 || index >= (int)path.size()-1){
        return 0 * radpm;
    }

    QLength radius = Math::circumradius(path[index-1], path[index], path[index+1]);

    if(std::isnan(radius.getValue())){
        return 0 * radpm;
    }

    return 1 * radian /radius;
}
}