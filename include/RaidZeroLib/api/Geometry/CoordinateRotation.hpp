#pragma once

namespace rz{

class CoordinateRotation{
    public:
    CoordinateRotation(double scale);

    static const CoordinateRotation& Clockwise();

    static const CoordinateRotation& Counterclockwise();

    protected:
    friend class CoordinateSystem;

    double scale;
};

}