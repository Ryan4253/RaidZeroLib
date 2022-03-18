#include "SimplePath.hpp"

namespace lib4253{

SimplePath::SimplePath(const std::initializer_list<Point>& iWaypoint):waypoint(iWaypoint){}

SimplePath& SimplePath::generate(int iStep, bool iEnd){
    if(iStep < 1){
        throw std::invalid_argument("SimplePath::generate(): step cannot be smaller than one");
    }

    std::vector<Point> path;
    for(int i = 0; i < waypoint.size()-1; i++){
         Point diff = waypoint[i+1]-waypoint[i];
         Point inc = diff/iStep;

        for(int j = 0; j < iStep; j++){
            path.push_back(waypoint[i]+(inc*j));
        }
    }

    if(waypoint.size() != 0 && iEnd){
        path.push_back(waypoint[waypoint.size()-1]);
    }

    waypoint = path;

    return *this;
}

SimplePath& SimplePath::generate(QLength iLength, bool iEnd){
    if(iLength == 0 * meter){
        throw std::invalid_argument("SimplePath::generate(): length cannot be 0");
    }

    std::vector<Point> path;
    for(int i = 0; i < waypoint.size()-1; i++){
        int step = std::ceil((waypoint[i].distTo(waypoint[i+1]) / iLength).convert(okapi::number)); 
        Point diff = waypoint[i+1] - waypoint[i];
        Point inc = diff / step;

        for(int j = 0; j < step; j++){
            path.push_back(waypoint[i]+(inc*j));
        }
    }

    if(waypoint.size() != 0 && iEnd){
        path.push_back(waypoint[waypoint.size()-1]);
    }

    waypoint = path;

    return *this;
}

DiscretePath SimplePath::smoothen(double iSmoothWeight, QLength iTolerance){
    okapi::QLength change = iTolerance;
    std::vector<Point> newPath = waypoint;
    double a = 1-iSmoothWeight, b = iSmoothWeight;

    while(change >= iTolerance){
        change = 0 * meter;
        for(int i = 1; i < waypoint.size()-1; i++){
            Point aux = newPath[i];

            newPath[i].setX(newPath[i].X() + a * (waypoint[i].X() - newPath[i].X()) + b * (newPath[i-1].X() +
            newPath[i+1].X() - (2.0 * newPath[i].X())));

            newPath[i].setY(newPath[i].Y() + a * (waypoint[i].Y() - newPath[i].Y()) + b * (newPath[i-1].Y() +
            newPath[i+1].Y() - (2.0 * newPath[i].Y())));

            change += abs(aux.X() + aux.Y() - newPath[i].X() - newPath[i].Y());
        }   
    }

    return DiscretePath(newPath);
}

DiscretePath SimplePath::noSmoothen(){
    return DiscretePath(waypoint);
}
}
