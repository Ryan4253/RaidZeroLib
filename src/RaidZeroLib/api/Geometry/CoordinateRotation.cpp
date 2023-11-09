#include "RaidZeroLib/api/Geometry/CoordinateRotation.hpp"

namespace rz{

CoordinateRotation::CoordinateRotation(double scale) : scale(scale){}

const CoordinateRotation& CoordinateRotation::Clockwise(){
    static const CoordinateRotation rot = CoordinateRotation(-1);
    return rot;
}

const CoordinateRotation& CoordinateRotation::Counterclockwise(){
    static const CoordinateRotation rot = CoordinateRotation(1);
    return rot;
}

}