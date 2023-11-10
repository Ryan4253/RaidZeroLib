#pragma once

namespace rz {

class CoordinateAxis {
    public:
    CoordinateAxis(double c1, double c2);

    static const CoordinateAxis& N();

    static const CoordinateAxis& S();

    static const CoordinateAxis& E();

    static const CoordinateAxis& W();

    protected:
    friend class CoordinateSystem;

    double c1, c2;
};

} // namespace rz