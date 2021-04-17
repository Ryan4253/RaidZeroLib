#pragma once
#include "main.h"
#include "Path.hpp"

namespace Math{
    double degToRad(double deg);
    double radToDeg(double rad);
    double tickToInch(double tick);
    double tickToInch(double tick, double rad, double ticksPerRotation);
    double inchToTick(double inch);
    double inchToTick(double tick, double rad, double ticksPerRotation);
    double cubicControl(double power);

    double wrapAngle360(double angle);
    double wrapAngle180(double angle);
    double wrapAngle90(double angle);

    Vector toPolar(Vector cart);
    Vector toCart(Vector polar);

    double clamp(double val, double mn, double mx);
};
