#include "lib4253/Trajectory/Spline/CompoundBezier.hpp"
namespace lib4253{

CompoundBezier::CompoundBezier(std::vector<Bezier> curve){
    for(int i = 0; i < curve.size()-1; i++){
        if(curve[i].getLastPoint() != curve[i+1].getFirstPoint()){
            throw std::runtime_error("CompoundBezier: path's endpoint does not match");
        }
    }
    path = curve;
}

Point2D CompoundBezier::at(double t){
    double compoundT = t * path.size();
    return path[(int)t].at(t-(int)t);
}

DiscretePath CompoundBezier::generate(int iStep, bool end){
    DiscretePath result;
    for(int i = 0; i < path.size(); i++){
        result += path[i].generate(iStep, false);
    }

    if(end){
        result += (path[path.size()-1].getLastPoint());
    }

    return result;
}

}