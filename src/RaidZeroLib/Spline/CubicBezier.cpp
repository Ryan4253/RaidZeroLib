#include "lib4253/Trajectory/Spline/CubicBezier.hpp"
namespace lib4253{


CubicBezier::Knot::Knot(QLength ix, QLength iy, QAngle iangle, QLength imag){
    x = ix; y = iy; angle = iangle; mag = imag;
}

CubicBezier::CubicBezier(const Point& iStart, const Point& iControl1, const Point& iControl2, const Point& iEnd){
    start = iStart; control1 = iControl1; control2 = iControl2; end = iEnd;
}

CubicBezier::CubicBezier(const Knot& iStart, const Knot& iEnd){
    start = Point(iStart.x, iStart.y);
    end = Point(iEnd.x, iEnd.y);
    control1 = start + Point(iStart.mag * cos(iStart.angle), iStart.mag * sin(iStart.angle));
    control2 = end + Point(iEnd.mag * cos(iEnd.angle + pi*radian), iEnd.mag * sin(iEnd.angle+pi*radian));
}

Point CubicBezier::getPoint(double t) const{
    return start*(1-t)*(1-t)*(1-t) + control1*3*(1-t)*(1-t)*t + control2*3*(1-t)*t*t + end*t*t*t;
}


DiscretePath CubicBezier::generate(int iStep, bool iEnd) const{
    if(iStep < 1){
        throw std::invalid_argument("CubicBezier::generate(): step cannot be smaller than 1");
    }

    std::vector<Point> path;
    path.reserve(iStep);
    double step = 1.0 / (iStep);
    for(int i = 0; i < iStep; i++){
        double t = i * step;
        path.emplace_back(getPoint(t));
    }

    if(iEnd){
        path.emplace_back(getPoint(1));
    }

    return DiscretePath(path);
}

Point CubicBezier::getVelocity(double t) const{
    return start*(-3*t*t+6*t-3) + control1*(9*t*t-12*t+3) + control2*(-9*t*t+6*t) + end*(3*t*t);
}

Point CubicBezier::getAcceleration(double t) const{
    return start*(-6*t+6) + control1*(18*t-12) + control2*(-18*t+6) + end*(6*t);
}

QCurvature CubicBezier::getCurvature(double t) const{
    Point v = getVelocity(t);
    Point a = getAcceleration(t);
    double vmag = v.mag().convert(meter);
    return radpm * (v.X().convert(meter) * a.Y().convert(meter) - v.Y().convert(meter) * a.X().convert(meter))/(vmag * vmag * vmag);
}

QLength CubicBezier::getLength() const{
    QLength result = 0*meter;
    for(double t = 0;  t < 1; t += 0.01){
        result += getPoint(t).distTo(getPoint(t+0.01));
    }

    return result;
}


}