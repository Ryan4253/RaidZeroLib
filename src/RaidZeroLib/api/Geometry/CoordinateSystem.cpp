#include "RaidZeroLib/api/Geometry/CoordinateSystem.hpp"

namespace rz{

CoordinateSystem::CoordinateSystem(const CoordinateAxis& iX,
                                   const CoordinateAxis& iY,
                                   const CoordinateRotation& iRotation) : x(iX), y(iY), rotation(iRotation){
if(x.c1 * y.c1 + x.c2 * y.c2 != 0){
    throw std::domain_error("CoordinateSystem: Provided axes are not orthogonal.");
}
}

const CoordinateSystem& CoordinateSystem::Standard(){
    static const auto system = CoordinateSystem(CoordinateAxis::E(),
                                                CoordinateAxis::N(),
                                                CoordinateRotation::Counterclockwise());
    return system;
}

Rotation CoordinateSystem::convert(const Rotation& iRotation, 
                                    const CoordinateSystem& from,
                                    const CoordinateSystem& to){
    return iRotation * from.rotation.scale / from.rotation.scale;
}

Point CoordinateSystem::convert(const Point& iPoint, 
                                const CoordinateSystem& from,
                                const CoordinateSystem& to){
    // Transforms point to standard basis
    const Point transform = Point(iPoint.X() * from.x.c1 + iPoint.Y() * from.y.c1,
                                    iPoint.X() * from.x.c2 + iPoint.Y() * from.y.c2);

    // Computes inverse matrix to transform to target basis
    const double determinant = to.x.c1 * to.y.c2 - to.y.c1 * to.x.c2;

    const double xc1 = to.y.c2 / determinant;
    const double xc2 = -to.x.c2 / determinant;
    const double yc1 = -to.y.c1 / determinant;
    const double yc2 = to.x.c1 / determinant;

    return Point(transform.X() * xc1 + transform.Y() * yc1,
                    transform.X() * xc2 + transform.Y() * yc2);
}

Pose CoordinateSystem::convert(const Pose& iPose, 
                               const CoordinateSystem& from,
                               const CoordinateSystem& to){
    return Pose(convert(iPose.getTranslation(), from, to), convert(iPose.getRotation(), from, to));
}

}