#pragma once
#include "RaidZeroLib/Geometry/Pose.hpp"
#include "RaidZeroLib/Geometry/CoordinateAxis.hpp"
#include "RaidZeroLib/Geometry/CoordinateRotation.hpp"
#include <stdexcept>

namespace rz{

class CoordinateSystem{
    public:
    CoordinateSystem(const CoordinateAxis& iX,
                     const CoordinateAxis& iY,
                     const CoordinateRotation& iRotation);
    
    static const CoordinateSystem& Standard();

    static Rotation convert(const Rotation& iRotation, 
                            const CoordinateSystem& from,
                            const CoordinateSystem& to = CoordinateSystem::Standard());

    static Point convert(const Point& iPoint, 
                         const CoordinateSystem& from,
                         const CoordinateSystem& to = CoordinateSystem::Standard());

    static Pose convert(const Pose& iPose, 
                        const CoordinateSystem& from,
                        const CoordinateSystem& to = CoordinateSystem::Standard());

    protected:
    CoordinateAxis x, y;
    CoordinateRotation rotation;
};

}