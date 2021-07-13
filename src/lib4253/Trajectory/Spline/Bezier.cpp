#include "lib4253/Trajectory/Spline/Bezier.hpp"
namespace lib4253{

Bezier::Bezier(std::vector<Point2D> control_point):waypoint(control_point){}

Point2D Bezier::getFirstPoint(){
    return waypoint[0];
}

Point2D Bezier::getLastPoint(){
    return waypoint[waypoint.size()-1];
}

Point2D Bezier::at(double t){
    Point2D res;
    int degree = waypoint.size()-1;
    for(int i = 0; i < waypoint.size(); i++){
        res = res + waypoint[i] * (Math::ncr(degree, i) * Math::power(1-t, degree-i) * Math::power(t, i));
    }
    return res;
}

DiscretePath Bezier::generate(int iStep, bool end){
    if(iStep < 1){
        throw std::runtime_error("Bezier - iStep < 0");
    }

    std::vector<Point2D> path;

    for(int i = 0; i < iStep; i++){
        path.push_back(at(1.0/iStep * i));
    }

    if(end){
        path.push_back(getLastPoint());
    }

    return DiscretePath(path);
}

/*
double Bezier::getLength(double tStart, double tEnd){
    double result = 0;
    std::stack<std::pair<double, double>> sta;
    sta.push(std::make_pair(tStart, tEnd));
    Point2D start, end;
    while(!sta.empty()){
        double t0 = sta.top().first;
        double t1 = sta.top().second;
        sta.pop();

        x   
    }
}
*/

}