#include "RaidZeroLib/api/Geometry/CoordinateAxis.hpp"

namespace rz{

CoordinateAxis::CoordinateAxis(double c1, double c2) : c1(c1), c2(c2){}

const CoordinateAxis& CoordinateAxis::N(){
    static const CoordinateAxis axis = CoordinateAxis(0, 1);
    return axis;
}

const CoordinateAxis& CoordinateAxis::S(){
    static const CoordinateAxis axis = CoordinateAxis(0, -1);
    return axis;
}

const CoordinateAxis& CoordinateAxis::E(){
    static const CoordinateAxis axis = CoordinateAxis(1, 0);
    return axis;
}

const CoordinateAxis& CoordinateAxis::W(){
    static const CoordinateAxis axis = CoordinateAxis(-1, 0);
    return axis;
}

}