#include "lib4253/Trajectory/Spline/DiscretePath.hpp"
namespace lib4253{

DiscretePath::DiscretePath(const std::initializer_list<Point2D>& waypoint){
    path = waypoint;
}

DiscretePath::DiscretePath(const std::vector<Point2D>& waypoint){
    path = waypoint;
}

DiscretePath DiscretePath::operator+(const DiscretePath& rhs) const{
    DiscretePath ret(path);
    ret += rhs;
    return ret;
}

DiscretePath DiscretePath::operator+(const Point2D& rhs) const{
    DiscretePath ret(path);
    ret += rhs;
    return ret;
}

DiscretePath& DiscretePath::operator+=(const DiscretePath& rhs){
    for(int i = 0; i < rhs.getSize(); i++){
        path.push_back(rhs[i]);
    }

    return *this;
}

DiscretePath& DiscretePath::operator+=(const Point2D& rhs){
    path.push_back(rhs);
    return *this;
}

Point2D DiscretePath::operator[](const int& index) const{
    if(index >= path.size() || index < 0){
        throw std::runtime_error("Discrete Path - Index out of bounds");
    }
    return path[index];
}

DiscretePath& DiscretePath::generate(int step, bool end){
    if(step < 1){
        throw std::runtime_error("DiscretePath:: step < 1");
    }

    std::vector<Point2D> newPath;;
    for(int i = 0; i < path.size()-1; i++){
         Point2D diff = path[i+1]-path[i];
         Point2D inc = diff/step;

        for(int j = 0; j < step; j++){
            newPath.push_back(path[i]+(inc*j));
        }
    }

    if(path.size() == 1 || end){
        newPath.push_back(path[path.size()-1]);
    }

    path = newPath;

    return *this;
}

DiscretePath& DiscretePath::generate(okapi::QLength dist, bool end){
    if(dist == 0 * okapi::meter){
        throw std::runtime_error("DiscretePath: dist == 0");
    }

    std::vector<Point2D> newPath;;
    for(int i = 0; i < path.size()-1; i++){
        int step = std::ceil((path[i].magnitude() / dist).convert(okapi::number)); 
        Point2D diff = path[i+1] - path[i];
        Point2D inc = diff / step;

        for(int j = 0; j < step; j++){
            newPath.push_back(path[i]+(inc*j));
        }
    }

    if(end){
        newPath.push_back(path[path.size()-1]);
    }

    path = newPath;

    return *this;
}

DiscretePath& DiscretePath::smooth(double a, double b, okapi::QLength tolerance){
    okapi::QLength change = tolerance;
    std::vector<Point2D> newPath = path;

    while(change >= tolerance){
        change = 0 * okapi::meter;
        for(int i = 1; i < path.size()-1; i++){
            Point2D aux = newPath[i];

            newPath[i].x = newPath[i].x + a * (path[i].x - newPath[i].x) + b * (newPath[i-1].x +
            newPath[i+1].x - (2.0 * newPath[i].x));

            newPath[i].y = newPath[i].y + a * (path[i].y - newPath[i].y) + b * (newPath[i-1].y +
            newPath[i+1].y - (2.0 * newPath[i].y));

            change += abs(aux.x + aux.y - newPath[i].x - newPath[i].y);
        }   
    }   
    path = newPath;

    return *this;
}

int DiscretePath::getSize() const{
    return path.size();
}

okapi::QCurvature DiscretePath::getCurvature(const int& index) const{
    if(index < 0 || index >= path.size()){
        throw std::runtime_error("Discrete Path - Index out of bounds");
    }
    else if(index == 0 || index == path.size()-1){
        return 0 * okapi::radpm;
    }
    else{
        okapi::QLength x1 = path[index].x + 0.001 * okapi::inch, y1 = path[index].y ;
        okapi::QLength x2 = path[index-1].x, y2 = path[index-1].y;
        okapi::QLength x3 = path[index+1].x, y3 = path[index+1].y;

        okapi::QLength k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1-x2);
        okapi::Number k2 = (y1-y2) / (x1-x2);
        okapi::QLength b = 0.5 * ((x2*x2)+(2*x2*k1)+(y2*y2)-(x3*x3)+(2*x3*k1)-(y3 * y3)) / ((x3*k2)-y3+y2-(x2*k2));
        okapi::QLength a = k1 - k2 * b;

        okapi::QLength r = sqrt((x1-a)*(x1-a)+(y1-b)*(y1-b));
        if(std::isnan(r.getValue())){
            return 0 * okapi::radpm;
        }
        else{
            return 1 * okapi::radian / r;
        }
    }
}
}