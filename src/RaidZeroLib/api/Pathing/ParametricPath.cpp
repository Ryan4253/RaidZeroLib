#include "RaidZeroLib/api/Pathing/ParametricPath.hpp"

namespace rz{

Rotation ParametricPath::getTheta(double t) const{
    return Rotation(getX(t), getY(t));
}

Point ParametricPath::getPoint(double t) const{
    return Point(getX(t), getY(t));
}

Point ParametricPath::getVelocity(double t) const{
    return Point(getdX(t), getdY(t));
}

Point ParametricPath::getAcceleration(double t) const{
    return Point(getddX(t), getddY(t));
}

QCurvature ParametricPath::getCurvature(double t) const{
    Point velocity = getVelocity(t);
    Point acceleration = getAcceleration(t);

    if(velocity.mag().getValue() == 0){
        return 0 * radpm;
    }

    return radian * velocity.wedge(acceleration) / cube(velocity.mag());
}

QLength ParametricPath::getLength(double tStart, double tEnd) const{
    QLength length{0.0};
    for(double t = tStart; t < tEnd; t += 0.01){
        length += getPoint(t).distTo(getPoint(t+0.01));
    }

    return length;
}

DiscretePath ParametricPath::toDiscrete(int numPoints, bool end) const{
    std::vector<Point> path;
    const double increment = 1.0 / (numPoints-1);

    for(double t = 0; t < 1; t += increment){
        path.emplace_back(getPoint(t));
    }

    if(end){
        path.emplace_back(getPoint(1));
    }

    return DiscretePath(path);
}

DiscretePath ParametricPath::toDiscrete(QLength distance, bool end) const{
    const QLength length = getLength();
    const QLength distPerSegment = length / ceil((length / distance).convert(number));

    QLength traversed{0.0};
    std::vector<Point> path; path.emplace_back(getPoint(0));

    for(double t = 0; t < 1; t += 0.01){
        traversed += getPoint(t).distTo(getPoint(t + 0.01));
        if(traversed >= distPerSegment){
            traversed = 0_m;
            path.emplace_back(getPoint(t));
        }
    }

    if(path.back().distTo(getPoint(1)) < distPerSegment / 2){
        path.pop_back();
    }

    if(end){
        path.emplace_back(getPoint(1));
    }

    return DiscretePath(path);
}

double ParametricPath::stepT(double t, QLength distance) const{
    return t + (distance / getVelocity(t).mag()).convert(number);
}

}